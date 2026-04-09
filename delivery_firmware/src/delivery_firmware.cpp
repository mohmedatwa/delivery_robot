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

// ─── Data structs ─────────────────────────────────────────────────────────────
struct WheelState {
  float fl = 0, fr = 0, bl = 0, br = 0;   // RPM
  unsigned long ts = 0;
};

// ─── Constants ────────────────────────────────────────────────────────────────
constexpr double RPM2RAD       = 0.104719755;
constexpr double RAD2RPM       = 9.5492968;
constexpr double CMD_THRESHOLD = 0.0;   // rad/s – skip tiny changes
constexpr int    WRITE_RATE_MS = 20;     // max TX rate  (50 Hz)
constexpr int    HEARTBEAT_MS  = 200;    // periodic resend
constexpr int    LOG_RATE_MS   = 1000;    // throttle repeat log lines

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
    last_sent_.assign(4, std::numeric_limits<double>::max()); // force first send

    auto p = [&](const char* k) -> std::string {
      auto it = info.hardware_parameters.find(k);
      return (it != info.hardware_parameters.end()) ? it->second : "";
    };
    if (!p("device").empty())    device_    = p("device");
    if (!p("baud_rate").empty()) baud_rate_ = std::stoi(p("baud_rate"));

    FW_INFO("Init: %s @ %d baud", device_.c_str(), baud_rate_);

    try {
      io_ctx_ = std::make_shared<boost::asio::io_context>();
      serial_ = std::make_unique<boost::asio::serial_port>(*io_ctx_);
      serial_->open(device_);
      serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
      serial_->set_option(boost::asio::serial_port_base::character_size(8));
      serial_->set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
      serial_->set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
      serial_->set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
      FW_INFO("Serial open OK");
    } catch (const std::exception & e) {
      FW_FATAL("Serial open failed: %s", e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // ── on_configure ──────────────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = false;
    read_thread_ = std::thread(&DeliveryFirmware::read_loop, this);
    std::this_thread::sleep_for(100ms);
    send("handshake\n");
    configured_ = true;
    FW_INFO("Configured");
    return CallbackReturn::SUCCESS;
  }

  // ── on_cleanup ────────────────────────────────────────────────────────────
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stop_read_ = true;
    if (read_thread_.joinable()) read_thread_.join();
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
    send("activate\n");
    FW_INFO("Activated");
    return CallbackReturn::SUCCESS;
  }

  // ── on_deactivate ─────────────────────────────────────────────────────────
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    send("stop\n");
    FW_INFO("Deactivated");
    return CallbackReturn::SUCCESS;
  }

  // ── export interfaces ─────────────────────────────────────────────────────
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
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (state_.ts > 0) {
        hw_velocities_[0] = state_.fl * RPM2RAD;
        hw_velocities_[1] = state_.fr * RPM2RAD;
        hw_velocities_[2] = state_.bl * RPM2RAD;
        hw_velocities_[3] = state_.br * RPM2RAD;
      }
    }

    double dt = period.seconds();
    for (size_t i = 0; i < 4; i++)
      hw_positions_[i] += hw_velocities_[i] * dt;

    FW_THROTTLE(LOG_RATE_MS, "VEL rad/s  FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
      hw_velocities_[0], hw_velocities_[1], hw_velocities_[2], hw_velocities_[3]);

    return hardware_interface::return_type::OK;
  }

  // ── write ─────────────────────────────────────────────────────────────────
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // if (!active_) return hardware_interface::return_type::OK;

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

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "FL:" << hw_commands_[0] * RAD2RPM
       << " FR:" << hw_commands_[1] * RAD2RPM
       << " BL:" << hw_commands_[2] * RAD2RPM
       << " BR:" << hw_commands_[3] * RAD2RPM << "\n";

    send(ss.str());

    last_sent_   = hw_commands_;
    t_write_     = now;
    t_heartbeat_ = now;

    FW_THROTTLE(LOG_RATE_MS, "CMD RPM  FL:%.1f FR:%.1f BL:%.1f BR:%.1f%s",
      hw_commands_[0]*RAD2RPM + 5, hw_commands_[1]*RAD2RPM ,
      hw_commands_[2]*RAD2RPM , hw_commands_[3]*RAD2RPM ,
      heartbeat ? " [hb]" : "");

    return hardware_interface::return_type::OK;
  }

private:

  // ── send ──────────────────────────────────────────────────────────────────
  void send(const std::string & msg)
  {
    std::lock_guard<std::mutex> lk(serial_mtx_);
    if (!serial_ || !serial_->is_open()) { FW_ERROR("Port not open"); return; }
    try {
      boost::asio::write(*serial_, boost::asio::buffer(msg));
      FW_DEBUG("TX: %s", msg.c_str());
    } catch (const std::exception & e) {
      FW_ERROR("TX error: %s", e.what());
    }
  }

  // ── read_loop ─────────────────────────────────────────────────────────────
  void read_loop()
  {
    boost::asio::streambuf buf;
    std::string line;
    FW_INFO("RX thread started");

    while (!stop_read_ && serial_ && serial_->is_open()) {
      try {
        boost::system::error_code ec;
        boost::asio::read_until(*serial_, buf, '\n', ec);

        if (ec == boost::asio::error::eof) { FW_WARN("Serial EOF"); break; }
        if (ec) throw boost::system::system_error(ec);

        std::istream is(&buf);
        std::getline(is, line);
        while (!line.empty() && (line.back()=='\r' || line.back()=='\n'))
          line.pop_back();

        if (!line.empty()) parse(line);

      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(LOG, *clock_, 2000, "RX error: %s", e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    FW_INFO("RX thread stopped");
  }

  // ── parse ─────────────────────────────────────────────────────────────────
  void parse(const std::string & line)
  {
    if (line.find("ESP:") != std::string::npos) {
      FW_INFO("ESP> %s", line.c_str() + line.find("ESP:") + 4);
      return;
    }

    WheelState s;
    bool ok = false;

    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" \t"), boost::token_compress_on);

    for (const auto & tok : tokens) {
      auto c = tok.find(':');
      if (c == std::string::npos) continue;
      std::string k = tok.substr(0, c), v = tok.substr(c + 1);
      try {
        if      (k=="FL") { s.fl = std::stof(v); ok = true; }
        else if (k=="FR") { s.fr = std::stof(v); ok = true; }
        else if (k=="BL") { s.bl = std::stof(v); ok = true; }
        else if (k=="BR") { s.br = std::stof(v); ok = true; }
        else if (k=="T")  { s.ts = std::stoul(v); }
      } catch (...) {
        FW_WARN("Bad token: %s", tok.c_str());
      }
    }

    if (ok) {
      std::lock_guard<std::mutex> lk(state_mtx_);
      state_ = s;
      FW_DEBUG("RX RPM  FL:%.1f FR:%.1f BL:%.1f BR:%.1f", s.fl, s.fr, s.bl, s.br);
    } else {
      FW_WARN("Unrecognised: %s", line.c_str());
    }
  }

  // ── helper ────────────────────────────────────────────────────────────────
  static long ms_since(const std::chrono::steady_clock::time_point & now,
                       const std::chrono::steady_clock::time_point & then)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count();
  }

  // ── members ───────────────────────────────────────────────────────────────
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_, last_sent_;

  std::string device_    = "/dev/ttyUSB0";
  int         baud_rate_ = 115200;

  std::shared_ptr<boost::asio::io_context>  io_ctx_;
  std::unique_ptr<boost::asio::serial_port> serial_;
  std::thread       read_thread_;
  std::mutex        serial_mtx_, state_mtx_;
  std::atomic<bool> stop_read_{false};

  WheelState state_;

  std::chrono::steady_clock::time_point t_write_, t_heartbeat_;
  bool configured_{false}, active_{false};

  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>();
};

// ── destructor ────────────────────────────────────────────────────────────────
DeliveryFirmware::~DeliveryFirmware()
{
  stop_read_ = true;
  if (read_thread_.joinable()) read_thread_.join();
  if (serial_ && serial_->is_open()) {
    try { send("stop\n"); serial_->close(); } catch (...) {}
  }
  if (io_ctx_) io_ctx_->stop();
  FW_INFO("Shutdown complete");
}

}  // namespace delivery_firmware

PLUGINLIB_EXPORT_CLASS(
  delivery_firmware::DeliveryFirmware,
  hardware_interface::SystemInterface)