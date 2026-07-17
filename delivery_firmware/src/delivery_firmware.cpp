#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <functional>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

namespace delivery_firmware
{
using namespace std::chrono_literals;
using CallbackReturn = hardware_interface::CallbackReturn;

// ─── Logger shorthand ────────────────────────────────────────────────────────
static auto LOG = rclcpp::get_logger("DeliveryFW");

#define FW_INFO(...)  RCLCPP_INFO (LOG, __VA_ARGS__)
#define FW_WARN(...)  RCLCPP_WARN (LOG, __VA_ARGS__)
#define FW_ERROR(...) RCLCPP_ERROR(LOG, __VA_ARGS__)
#define FW_FATAL(...) RCLCPP_FATAL(LOG, __VA_ARGS__)
#define FW_DEBUG(...) RCLCPP_DEBUG(LOG, __VA_ARGS__)
#define FW_THROTTLE(ms, ...) \
  RCLCPP_INFO_THROTTLE(LOG, *clock_, ms, __VA_ARGS__)

// ─── Unified motor state ──────────────────────────────────────────────────────
// كل موتور له قيمة RPM مستقلة + timestamp آخر تحديث
// يُحدَّث من أي USB وصلت منه البيانات
struct MotorState {
  float    fl = 0.0f;   // Front-Left  RPM
  float    fr = 0.0f;   // Front-Right RPM
  float    bl = 0.0f;   // Back-Left   RPM
  float    br = 0.0f;   // Back-Right  RPM
  unsigned long ts_fl = 0;
  unsigned long ts_fr = 0;
  unsigned long ts_bl = 0;
  unsigned long ts_br = 0;
};

// ─── Constants ────────────────────────────────────────────────────────────────
constexpr double RPM2RAD       = 0.104719755;
constexpr double RAD2RPM       = 9.5492968;
constexpr double CMD_THRESHOLD = 0.0;
constexpr int    WRITE_RATE_MS = 20;
constexpr int    HEARTBEAT_MS  = 200;
constexpr int    LOG_RATE_MS   = 1000;

// ─── Serial port wrapper ──────────────────────────────────────────────────────
struct SerialPort
{
  std::shared_ptr<boost::asio::io_context>  io_ctx;
  std::unique_ptr<boost::asio::serial_port> port;
  std::mutex                                mtx;

  bool open(const std::string & device, int baud)
  {
    try {
      io_ctx = std::make_shared<boost::asio::io_context>();
      port   = std::make_unique<boost::asio::serial_port>(*io_ctx);
      port->open(device);
      port->set_option(boost::asio::serial_port_base::baud_rate(baud));
      port->set_option(boost::asio::serial_port_base::character_size(8));
      port->set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
      port->set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
      port->set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
      return true;
    } catch (...) { return false; }
  }

  void send(const std::string & msg)
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (!port || !port->is_open()) return;
    try {
      boost::asio::write(*port, boost::asio::buffer(msg));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("DeliveryFW"), "TX error: %s", e.what());
    }
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (port && port->is_open()) {
      try { port->close(); } catch (...) {}
    }
    if (io_ctx) io_ctx->stop();
  }

  bool is_open() const { return port && port->is_open(); }
};

// ─────────────────────────────────────────────────────────────────────────────
class DeliveryFirmware : public hardware_interface::SystemInterface
{
public:
  DeliveryFirmware()  = default;
  ~DeliveryFirmware() override;

  // ── on_init ────────────────────────────────────────────────────────────────
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    const auto & info = get_hardware_info();
    if (info.joints.size() != 4) {
      FW_FATAL("Need 4 joints, got %zu", info.joints.size());
      return CallbackReturn::ERROR;
    }

    hw_positions_.assign(4, 0.0);
    hw_velocities_.assign(4, 0.0);
    hw_commands_.assign(4, 0.0);
    last_sent_.assign(4, std::numeric_limits<double>::max());

    auto p = [&](const char* k, const char* def) -> std::string {
      auto it = info.hardware_parameters.find(k);
      return (it != info.hardware_parameters.end()) ? it->second : def;
    };

    baud_rate_ = std::stoi(p("baud_rate", "115200"));

    // ── قراءة المنفذين مباشرةً من URDF/YAML ─────────────────────────────
    // usb0_device → منفذ أول   (يُبعَت عليه الأوامر + يُستقبَل منه)
    // usb1_device → منفذ تاني  (يُبعَت عليه الأوامر + يُستقبَل منه)
    usb0_device_ = p("usb0_device", "/dev/ttyUSB0");
    usb1_device_ = p("usb1_device", "/dev/ttyUSB1");

    FW_INFO("USB0: %s  |  USB1: %s  |  baud: %d",
            usb0_device_.c_str(), usb1_device_.c_str(), baud_rate_);

    // ── افتح المنفذ الأول ─────────────────────────────────────────────────
    if (!usb0_.open(usb0_device_, baud_rate_)) {
      FW_FATAL("Failed to open USB0: %s", usb0_device_.c_str());
      return CallbackReturn::ERROR;
    }
    FW_INFO("USB0 serial open OK");

    // ── افتح المنفذ التاني ────────────────────────────────────────────────
    if (!usb1_.open(usb1_device_, baud_rate_)) {
      FW_FATAL("Failed to open USB1: %s", usb1_device_.c_str());
      usb0_.close();
      return CallbackReturn::ERROR;
    }
    FW_INFO("USB1 serial open OK");

    FW_INFO("Both ports open. Init complete.");
    return CallbackReturn::SUCCESS;
  }

  // ── on_configure ──────────────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = false;

    // thread واحد لكل USB – كلهم يكتبوا في motor_state_ الموحد
    read_usb0_thread_ = std::thread(&DeliveryFirmware::read_loop_usb0, this);
    read_usb1_thread_ = std::thread(&DeliveryFirmware::read_loop_usb1, this);

    std::this_thread::sleep_for(100ms);

    // أرسل handshake للاتنين
    usb0_.send("handshake\n");
    usb1_.send("handshake\n");

    configured_ = true;
    FW_INFO("Configured – broadcasting to both USB ports");
    return CallbackReturn::SUCCESS;
  }

  // ── on_cleanup ────────────────────────────────────────────────────────────
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = true;
    if (read_usb0_thread_.joinable()) read_usb0_thread_.join();
    if (read_usb1_thread_.joinable()) read_usb1_thread_.join();
    configured_ = false;
    FW_INFO("Cleaned up");
    return CallbackReturn::SUCCESS;
  }

  // ── on_activate ───────────────────────────────────────────────────────────
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!configured_) { FW_FATAL("Not configured"); return CallbackReturn::ERROR; }
    std::fill(hw_commands_.begin(),   hw_commands_.end(),   0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    std::fill(hw_positions_.begin(),  hw_positions_.end(),  0.0);
    std::fill(last_sent_.begin(),     last_sent_.end(), std::numeric_limits<double>::max());
    t_write_ = t_heartbeat_ = std::chrono::steady_clock::now();
    active_ = true;
    usb0_.send("activate\n");
    usb1_.send("activate\n");
    FW_INFO("Activated");
    return CallbackReturn::SUCCESS;
  }

  // ── on_deactivate ─────────────────────────────────────────────────────────
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    usb0_.send("stop\n");
    usb1_.send("stop\n");
    FW_INFO("Deactivated");
    return CallbackReturn::SUCCESS;
  }

  // ── export interfaces ─────────────────────────────────────────────────────
  // Joint order: 0=FL, 1=FR, 2=BL, 3=BR  (must match URDF)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> v;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return v;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> v;
    for (size_t i = 0; i < info_.joints.size(); i++)
      v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    return v;
  }

  // ── read ──────────────────────────────────────────────────────────────────
  // يقرأ من motor_state_ الموحد – الـ timestamp يضمن إن القيمة اتحدثت
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    {
      std::lock_guard<std::mutex> lk(motor_state_mtx_);
      if (motor_state_.ts_fl > 0) hw_velocities_[0] = motor_state_.fl * RPM2RAD;
      if (motor_state_.ts_fr > 0) hw_velocities_[1] = motor_state_.fr * RPM2RAD;
      if (motor_state_.ts_bl > 0) hw_velocities_[2] = motor_state_.bl * RPM2RAD;
      if (motor_state_.ts_br > 0) hw_velocities_[3] = motor_state_.br * RPM2RAD;
    }

    double dt = period.seconds();
    for (size_t i = 0; i < 4; i++)
      hw_positions_[i] += hw_velocities_[i] * dt;

    FW_THROTTLE(LOG_RATE_MS, "VEL rad/s  FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
      hw_velocities_[0], hw_velocities_[1], hw_velocities_[2], hw_velocities_[3]);

    return hardware_interface::return_type::OK;
  }

  // ── write ─────────────────────────────────────────────────────────────────
  // يبعت أوامر الـ 4 مواتير على الـ USB تنين في نفس الوقت
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto now = std::chrono::steady_clock::now();
    if (ms_since(now, t_write_) < WRITE_RATE_MS)
      return hardware_interface::return_type::OK;

    bool changed = false;
    for (size_t i = 0; i < 4; i++)
      if (std::abs(hw_commands_[i] - last_sent_[i]) > CMD_THRESHOLD)
        { changed = true; break; }

    bool heartbeat = ms_since(now, t_heartbeat_) >= HEARTBEAT_MS;
    if (!changed && !heartbeat)
      return hardware_interface::return_type::OK;

    // ── بناء رسالة واحدة تحتوي الـ 4 مواتير ─────────────────────────────
    // الصيغة: "FL:xx.xx FR:xx.xx BL:xx.xx BR:xx.xx\n"
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "FL:" << hw_commands_[0] * RAD2RPM
       << " FR:" << hw_commands_[1] * RAD2RPM
       << " BL:" << hw_commands_[2] * RAD2RPM
       << " BR:" << hw_commands_[3] * RAD2RPM
       << "\n";
    std::string msg = ss.str();

    // ── أرسل على الـ USB تنين ─────────────────────────────────────────────
    usb0_.send(msg);
    usb1_.send(msg);

    last_sent_   = hw_commands_;
    t_write_     = now;
    t_heartbeat_ = now;

    FW_THROTTLE(LOG_RATE_MS, "CMD RPM  FL:%.1f FR:%.1f BL:%.1f BR:%.1f%s",
      hw_commands_[0]*RAD2RPM, hw_commands_[1]*RAD2RPM,
      hw_commands_[2]*RAD2RPM, hw_commands_[3]*RAD2RPM,
      heartbeat ? " [hb]" : "");

    return hardware_interface::return_type::OK;
  }

private:

  // ── read_loop_usb0 ────────────────────────────────────────────────────────
  void read_loop_usb0()
  {
    boost::asio::streambuf buf;
    std::string line;
    FW_INFO("RX-USB0 thread started (%s)", usb0_device_.c_str());

    while (!stop_read_ && usb0_.is_open()) {
      try {
        boost::system::error_code ec;
        boost::asio::read_until(*usb0_.port, buf, '\n', ec);

        if (ec == boost::asio::error::eof) { FW_WARN("USB0 serial EOF"); break; }
        if (ec) throw boost::system::system_error(ec);

        std::istream is(&buf);
        std::getline(is, line);
        strip_crlf(line);
        if (!line.empty()) parse_motor_line(line, "USB0");

      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(LOG, *clock_, 2000, "RX-USB0 error: %s", e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    FW_INFO("RX-USB0 thread stopped");
  }

  // ── read_loop_usb1 ────────────────────────────────────────────────────────
  void read_loop_usb1()
  {
    boost::asio::streambuf buf;
    std::string line;
    FW_INFO("RX-USB1 thread started (%s)", usb1_device_.c_str());

    while (!stop_read_ && usb1_.is_open()) {
      try {
        boost::system::error_code ec;
        boost::asio::read_until(*usb1_.port, buf, '\n', ec);

        if (ec == boost::asio::error::eof) { FW_WARN("USB1 serial EOF"); break; }
        if (ec) throw boost::system::system_error(ec);

        std::istream is(&buf);
        std::getline(is, line);
        strip_crlf(line);
        if (!line.empty()) parse_motor_line(line, "USB1");

      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(LOG, *clock_, 2000, "RX-USB1 error: %s", e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    FW_INFO("RX-USB1 thread stopped");
  }

  // ── parse_motor_line ──────────────────────────────────────────────────────
  // يُحلِّل أي سطر قادم من أي USB
  // يقبل أي token من: FL / FR / BL / BR
  // ويكتبه في motor_state_ الموحد
  //
  // مثال على السطر:
  //   "FL:120.5 T:54321"
  //   "BL:-45.0 BR:-44.8 T:54322"
  //   "FL:100.0 FR:100.0 BL:98.0 BR:99.0 T:54323"
  void parse_motor_line(const std::string & line, const char* src)
  {
    // رسائل debug من الـ ESP32 – تجاهل
    if (line.find("ESP:") != std::string::npos) {
      FW_INFO("[%s] ESP> %s", src,
              line.c_str() + line.find("ESP:") + 4);
      return;
    }

    // فصّل الـ tokens
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" \t"), boost::token_compress_on);

    bool        any_motor = false;
    float       val_fl = 0, val_fr = 0, val_bl = 0, val_br = 0;
    bool        got_fl = false, got_fr = false, got_bl = false, got_br = false;
    unsigned long ts = 0;

    for (const auto & tok : tokens) {
      auto c = tok.find(':');
      if (c == std::string::npos) continue;

      std::string k = tok.substr(0, c);
      std::string v = tok.substr(c + 1);

      try {
        if      (k == "FL") { val_fl = std::stof(v); got_fl = true; any_motor = true; }
        else if (k == "FR") { val_fr = std::stof(v); got_fr = true; any_motor = true; }
        else if (k == "BL") { val_bl = std::stof(v); got_bl = true; any_motor = true; }
        else if (k == "BR") { val_br = std::stof(v); got_br = true; any_motor = true; }
        else if (k == "T")  { ts     = std::stoul(v); }
      } catch (...) {
        FW_WARN("[%s] bad token: %s", src, tok.c_str());
      }
    }

    if (!any_motor) {
      FW_WARN("[%s] unrecognised line: %s", src, line.c_str());
      return;
    }

    // اعمل timestamp لو مجاش من الـ ESP
    if (ts == 0)
      ts = static_cast<unsigned long>(
        std::chrono::steady_clock::now().time_since_epoch().count());

    // اكتب في الـ state الموحد – كل موتور بـ timestamp مستقل
    {
      std::lock_guard<std::mutex> lk(motor_state_mtx_);
      if (got_fl) { motor_state_.fl = val_fl; motor_state_.ts_fl = ts; }
      if (got_fr) { motor_state_.fr = val_fr; motor_state_.ts_fr = ts; }
      if (got_bl) { motor_state_.bl = val_bl; motor_state_.ts_bl = ts; }
      if (got_br) { motor_state_.br = val_br; motor_state_.ts_br = ts; }
    }

    FW_DEBUG("[%s] RX RPM%s%s%s%s",
      src,
      got_fl ? (std::string(" FL:") + std::to_string(val_fl)).c_str() : "",
      got_fr ? (std::string(" FR:") + std::to_string(val_fr)).c_str() : "",
      got_bl ? (std::string(" BL:") + std::to_string(val_bl)).c_str() : "",
      got_br ? (std::string(" BR:") + std::to_string(val_br)).c_str() : "");
  }

  // ── helpers ───────────────────────────────────────────────────────────────
  static void strip_crlf(std::string & s)
  {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n'))
      s.pop_back();
  }

  static long ms_since(const std::chrono::steady_clock::time_point & now,
                       const std::chrono::steady_clock::time_point & then)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count();
  }

  // ── members ───────────────────────────────────────────────────────────────
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_, last_sent_;

  std::string usb0_device_;   // /dev/ttyUSB0  (من YAML)
  std::string usb1_device_;   // /dev/ttyUSB1  (من YAML)
  int         baud_rate_ = 115200;

  SerialPort usb0_;
  SerialPort usb1_;

  std::thread       read_usb0_thread_;
  std::thread       read_usb1_thread_;
  std::atomic<bool> stop_read_{false};

  // ── State موحد لكل المواتير ───────────────────────────────────────────────
  MotorState motor_state_;
  std::mutex motor_state_mtx_;

  std::chrono::steady_clock::time_point t_write_, t_heartbeat_;
  bool configured_{false}, active_{false};

  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>();
};

// ── destructor ────────────────────────────────────────────────────────────────
DeliveryFirmware::~DeliveryFirmware()
{
  stop_read_ = true;
  if (read_usb0_thread_.joinable()) read_usb0_thread_.join();
  if (read_usb1_thread_.joinable()) read_usb1_thread_.join();

  usb0_.send("stop\n");
  usb1_.send("stop\n");
  usb0_.close();
  usb1_.close();

  FW_INFO("Shutdown complete");
}

}  // namespace delivery_firmware

PLUGINLIB_EXPORT_CLASS(
  delivery_firmware::DeliveryFirmware,
  hardware_interface::SystemInterface)