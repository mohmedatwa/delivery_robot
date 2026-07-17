// arm_hardware_interface.cpp
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
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

namespace arm_hardware
{
using namespace std::chrono_literals;
using CallbackReturn = hardware_interface::CallbackReturn;

static auto LOG = rclcpp::get_logger("ArmHW");
#define HW_INFO(...)  RCLCPP_INFO (LOG, __VA_ARGS__)
#define HW_WARN(...)  RCLCPP_WARN (LOG, __VA_ARGS__)
#define HW_ERROR(...) RCLCPP_ERROR(LOG, __VA_ARGS__)
#define HW_FATAL(...) RCLCPP_FATAL(LOG, __VA_ARGS__)
#define HW_THROTTLE(ms, ...) \
  RCLCPP_INFO_THROTTLE(LOG, *clock_, ms, __VA_ARGS__)

// ─── Constants ────────────────────────────────────────────
constexpr int    NUM_JOINTS    = 6;
constexpr double RAD2DEG       = 57.2957795;
constexpr double DEG2RAD       = 0.01745329;
constexpr double SERVO_OFFSET  = 90.0;   // rad 0 → deg 90
constexpr int    WRITE_RATE_MS = 20;     // 50 Hz
constexpr int    HEARTBEAT_MS  = 200;
constexpr int    LOG_RATE_MS   = 1000;
constexpr double CMD_THRESHOLD = 0.001;  // rad — تجاهل تغيير أصغر من 0.001 rad

// ─── Serial port wrapper ──────────────────────────────────
struct SerialPort
{
  std::shared_ptr<boost::asio::io_context>  io_ctx;
  std::unique_ptr<boost::asio::serial_port> port;
  std::mutex mtx;

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
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("ArmHW"),
                   "Serial open failed: %s", e.what());
      return false;
    }
  }

  void send(const std::string & msg)
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (!port || !port->is_open()) return;
    try {
      boost::asio::write(*port, boost::asio::buffer(msg));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHW"), "TX error: %s", e.what());
    }
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (port && port->is_open()) try { port->close(); } catch (...) {}
    if (io_ctx) io_ctx->stop();
  }

  bool is_open() const { return port && port->is_open(); }
};

// ─────────────────────────────────────────────────────────
class ArmHardware : public hardware_interface::SystemInterface
{
public:
  ArmHardware()  = default;
  ~ArmHardware() override;

  // ── on_init ──────────────────────────────────────────────
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    const auto & info = get_hardware_info();
    if (info.joints.size() != NUM_JOINTS) {
      HW_FATAL("Expected %d joints, got %zu", NUM_JOINTS, info.joints.size());
      return CallbackReturn::ERROR;
    }

    hw_positions_.assign(NUM_JOINTS, 0.0);   // rad — state
    hw_commands_.assign (NUM_JOINTS, 0.0);   // rad — command
    last_sent_.assign   (NUM_JOINTS, std::numeric_limits<double>::max());
    fb_positions_.assign(NUM_JOINTS, 0.0);   // rad — feedback from ESP

    auto p = [&](const char* k, const char* def) -> std::string {
      auto it = info.hardware_parameters.find(k);
      return (it != info.hardware_parameters.end()) ? it->second : def;
    };

    serial_device_ = p("serial_port", "/dev/ttyUSB0");
    baud_rate_     = std::stoi(p("baud_rate", "115200"));

    HW_INFO("Serial: %s @ %d baud", serial_device_.c_str(), baud_rate_);

    if (!serial_.open(serial_device_, baud_rate_)) {
      HW_FATAL("Cannot open %s", serial_device_.c_str());
      return CallbackReturn::ERROR;
    }
    HW_INFO("Serial port open OK");
    return CallbackReturn::SUCCESS;
  }

  // ── on_configure ─────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = false;
    read_thread_ = std::thread(&ArmHardware::read_loop, this);
    std::this_thread::sleep_for(100ms);
    serial_.send("handshake\n");
    configured_ = true;
    HW_INFO("Configured");
    return CallbackReturn::SUCCESS;
  }

  // ── on_cleanup ────────────────────────────────────────────
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = true;
    if (read_thread_.joinable()) read_thread_.join();
    configured_ = false;
    HW_INFO("Cleaned up");
    return CallbackReturn::SUCCESS;
  }

  // ── on_activate ───────────────────────────────────────────
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!configured_) { HW_FATAL("Not configured"); return CallbackReturn::ERROR; }

    // ابدأ من الـ feedback الحالي — تجنب قفزة مفاجئة
    {
      std::lock_guard<std::mutex> lk(fb_mtx_);
      hw_positions_ = fb_positions_;
      hw_commands_  = fb_positions_;
    }
    std::fill(last_sent_.begin(), last_sent_.end(),
              std::numeric_limits<double>::max());

    t_write_ = t_heartbeat_ = std::chrono::steady_clock::now();
    active_ = true;
    serial_.send("activate\n");
    HW_INFO("Activated");
    return CallbackReturn::SUCCESS;
  }

  // ── on_deactivate ─────────────────────────────────────────
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    serial_.send("stop\n");
    HW_INFO("Deactivated");
    return CallbackReturn::SUCCESS;
  }

  // ── export interfaces ─────────────────────────────────────
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> v;
    for (size_t i = 0; i < NUM_JOINTS; i++)
      v.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION,
                     &hw_positions_[i]);
    return v;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> v;
    for (size_t i = 0; i < NUM_JOINTS; i++)
      v.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION,
                     &hw_commands_[i]);
    return v;
  }

  // ── read ──────────────────────────────────────────────────
  // نسخ الـ feedback من الـ read thread → hw_positions_
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::lock_guard<std::mutex> lk(fb_mtx_);
    for (int i = 0; i < NUM_JOINTS; i++)
      if (fb_updated_[i]) {
        hw_positions_[i] = fb_positions_[i];
        fb_updated_[i]   = false;
      }

    HW_THROTTLE(LOG_RATE_MS,
      "POS(rad) J0:%.2f J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f",
      hw_positions_[0], hw_positions_[1], hw_positions_[2],
      hw_positions_[3], hw_positions_[4], hw_positions_[5]);

    return hardware_interface::return_type::OK;
  }

  // ── write ─────────────────────────────────────────────────
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto now = std::chrono::steady_clock::now();
    if (ms_since(now, t_write_) < WRITE_RATE_MS)
      return hardware_interface::return_type::OK;

    bool changed = false;
    for (int i = 0; i < NUM_JOINTS; i++)
      if (std::abs(hw_commands_[i] - last_sent_[i]) > CMD_THRESHOLD)
        { changed = true; break; }

    bool heartbeat = ms_since(now, t_heartbeat_) >= HEARTBEAT_MS;
    if (!changed && !heartbeat)
      return hardware_interface::return_type::OK;

    // ── بناء الرسالة ─────────────────────────────────────
    // ROS radians (-π/2 → +π/2)  →  degrees (0 → 180)
    // deg = rad * RAD2DEG + 90
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    for (int i = 0; i < NUM_JOINTS; i++) {
      double deg = hw_commands_[i] * RAD2DEG + SERVO_OFFSET;
      deg = std::clamp(deg, 0.0, 180.0);          // hard limit
      ss << "J" << i << ":" << deg;
      if (i < NUM_JOINTS - 1) ss << " ";
    }
    ss << "\n";

    serial_.send(ss.str());
    last_sent_   = hw_commands_;
    t_write_     = now;
    t_heartbeat_ = now;

    HW_THROTTLE(LOG_RATE_MS,
      "CMD(deg) J0:%.1f J1:%.1f J2:%.1f J3:%.1f J4:%.1f J5:%.1f%s",
      hw_commands_[0]*RAD2DEG+90, hw_commands_[1]*RAD2DEG+90,
      hw_commands_[2]*RAD2DEG+90, hw_commands_[3]*RAD2DEG+90,
      hw_commands_[4]*RAD2DEG+90, hw_commands_[5]*RAD2DEG+90,
      heartbeat ? " [hb]" : "");

    return hardware_interface::return_type::OK;
  }

private:

  // ── read_loop ─────────────────────────────────────────────
  void read_loop()
  {
    boost::asio::streambuf buf;
    std::string line;
    HW_INFO("RX thread started (%s)", serial_device_.c_str());

    while (!stop_read_ && serial_.is_open()) {
      try {
        boost::system::error_code ec;
        boost::asio::read_until(*serial_.port, buf, '\n', ec);

        if (ec == boost::asio::error::eof) { HW_WARN("Serial EOF"); break; }
        if (ec) throw boost::system::system_error(ec);

        std::istream is(&buf);
        std::getline(is, line);
        strip_crlf(line);
        if (!line.empty()) parse_feedback(line);

      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(LOG, *clock_, 2000,
                              "RX error: %s", e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    HW_INFO("RX thread stopped");
  }

  // ── parse_feedback ────────────────────────────────────────
  // يُحلِّل: "J0:90.00 J1:45.00 J2:90.00 J3:90.00 J4:90.00 J5:45.00"
  // يحول degrees → radians ويحفظ في fb_positions_
  void parse_feedback(const std::string & line)
  {
    // تجاهل رسائل debug من الـ ESP
    if (line.rfind("DBG:", 0) == 0) {
      HW_INFO("ESP> %s", line.c_str() + 4);
      return;
    }

    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" \t"),
                 boost::token_compress_on);

    std::lock_guard<std::mutex> lk(fb_mtx_);
    for (const auto & tok : tokens) {
      if (tok.size() < 3) continue;
      // يقبل J0..J5
      if (tok[0] != 'J') continue;
      int idx = tok[1] - '0';
      if (idx < 0 || idx >= NUM_JOINTS) continue;
      auto c = tok.find(':');
      if (c == std::string::npos) continue;
      try {
        double deg = std::stod(tok.substr(c + 1));
        deg = std::clamp(deg, 0.0, 180.0);
        fb_positions_[idx] = (deg - SERVO_OFFSET) * DEG2RAD;
        fb_updated_[idx]   = true;
      } catch (...) {
        HW_WARN("Bad token: %s", tok.c_str());
      }
    }
  }

  // ── helpers ───────────────────────────────────────────────
  static void strip_crlf(std::string & s)
  {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n'))
      s.pop_back();
  }

  static long ms_since(
    const std::chrono::steady_clock::time_point & now,
    const std::chrono::steady_clock::time_point & then)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      now - then).count();
  }

  // ── members ───────────────────────────────────────────────
  std::string serial_device_;
  int         baud_rate_ = 115200;
  SerialPort  serial_;

  std::vector<double> hw_positions_;   // state  (rad)
  std::vector<double> hw_commands_;    // command (rad)
  std::vector<double> last_sent_;

  // feedback من الـ ESP — يُكتب من read thread، يُقرأ من read()
  std::vector<double>  fb_positions_;
  std::vector<bool>    fb_updated_ = std::vector<bool>(NUM_JOINTS, false);
  std::mutex           fb_mtx_;

  std::thread       read_thread_;
  std::atomic<bool> stop_read_{false};

  std::chrono::steady_clock::time_point t_write_, t_heartbeat_;
  bool configured_{false}, active_{false};

  std::shared_ptr<rclcpp::Clock> clock_ =
    std::make_shared<rclcpp::Clock>();
};

// ── destructor ────────────────────────────────────────────
ArmHardware::~ArmHardware()
{
  stop_read_ = true;
  if (read_thread_.joinable()) read_thread_.join();
  serial_.send("stop\n");
  serial_.close();
  HW_INFO("Shutdown complete");
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(
  arm_hardware::ArmHardware,
  hardware_interface::SystemInterface)