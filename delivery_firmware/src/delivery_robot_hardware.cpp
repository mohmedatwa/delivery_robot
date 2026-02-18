#include "delivery_firmware/delivery_robot_hardware.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace delivery_robot
{

// Helper to split a comma-separated string and trim whitespace
static std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ',')) {
    // Trim leading/trailing spaces
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

hardware_interface::return_type DeliveryRobotHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  // Call base class configure (stores info_)
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  ports_.clear();

  // Loop over port indices until no more "port<N>" parameters are found
  int port_idx = 1;
  while (true) {
    std::string port_key = "port" + std::to_string(port_idx);
    auto it = info_.hardware_parameters.find(port_key);
    if (it == info_.hardware_parameters.end()) {
      break;  // no more ports
    }

    MotorPort port;
    port.device = it->second;

    // Baud rate (optional, default 115200)
    std::string baud_key = "baud" + std::to_string(port_idx);
    auto baud_it = info_.hardware_parameters.find(baud_key);
    if (baud_it != info_.hardware_parameters.end()) {
      port.baudrate = std::stoi(baud_it->second);
    } else {
      port.baudrate = 115200;
    }

    // Joints assigned to this port
    std::string joints_key = "joints_port" + std::to_string(port_idx);
    auto joints_it = info_.hardware_parameters.find(joints_key);
    if (joints_it == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Missing parameter '%s' for port %d", joints_key.c_str(), port_idx);
      return hardware_interface::return_type::ERROR;
    }
    port.joint_names = split(joints_it->second);

    // Motor IDs for this port (optional, defaults to 0,1,2,...)
    std::string ids_key = "motor_ids_port" + std::to_string(port_idx);
    auto ids_it = info_.hardware_parameters.find(ids_key);
    if (ids_it != info_.hardware_parameters.end()) {
      std::vector<std::string> id_strs = split(ids_it->second);
      for (const auto & s : id_strs) {
        port.motor_ids.push_back(std::stoi(s));
      }
    } else {
      // Default IDs: 0,1,2,... for each joint on this port
      for (size_t i = 0; i < port.joint_names.size(); ++i) {
        port.motor_ids.push_back(static_cast<int>(i));
      }
    }

    // Check consistency
    if (port.joint_names.size() != port.motor_ids.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Joint count (%zu) and motor ID count (%zu) mismatch on port %s",
        port.joint_names.size(), port.motor_ids.size(), port.device.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Resize per‑port storage
    port.cmd_vel.resize(port.joint_names.size(), 0.0);
    port.pos.resize(port.joint_names.size(), 0.0);
    port.vel.resize(port.joint_names.size(), 0.0);

    // Open serial port
    try {
      port.serial = std::make_shared<serial::Serial>(
        port.device,
        port.baudrate,
        serial::Timeout::simpleTimeout(100)  // 100 ms read timeout
      );
      if (!port.serial->isOpen()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("DeliveryRobotHardware"),
          "Failed to open serial port %s", port.device.c_str());
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Opened serial port %s at %d baud", port.device.c_str(), port.baudrate);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Exception opening %s: %s", port.device.c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }

    ports_.push_back(std::move(port));
    ++port_idx;
  }

  if (ports_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DeliveryRobotHardware"),
      "No serial ports defined in hardware parameters");
    return hardware_interface::return_type::ERROR;
  }

  // Build mapping from global joint index to (port_index, local_index)
  joint_to_port_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    bool found = false;
    for (size_t p = 0; p < ports_.size(); ++p) {
      auto it = std::find(
        ports_[p].joint_names.begin(),
        ports_[p].joint_names.end(),
        joint_name);
      if (it != ports_[p].joint_names.end()) {
        size_t local_idx = std::distance(ports_[p].joint_names.begin(), it);
        joint_to_port_[i] = {p, local_idx};
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Joint '%s' is not assigned to any serial port", joint_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DeliveryRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto [port_idx, local_idx] = joint_to_port_[i];
    state_interfaces.emplace_back(
      info_.joints[i].name, "position", &ports_[port_idx].pos[local_idx]);
    state_interfaces.emplace_back(
      info_.joints[i].name, "velocity", &ports_[port_idx].vel[local_idx]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DeliveryRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto [port_idx, local_idx] = joint_to_port_[i];
    command_interfaces.emplace_back(
      info_.joints[i].name, "velocity", &ports_[port_idx].cmd_vel[local_idx]);
  }
  return command_interfaces;
}

hardware_interface::return_type DeliveryRobotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("DeliveryRobotHardware"), "Starting hardware...");
  // Enable motors, etc., if needed
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeliveryRobotHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("DeliveryRobotHardware"), "Stopping hardware...");
  for (auto & port : ports_) {
    if (port.serial && port.serial->isOpen()) {
      port.serial->close();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeliveryRobotHardware::read()
{
  for (auto & port : ports_) {
    // -----------------------------------------------------------------
    // !!! REPLACE THIS SECTION WITH YOUR ACTUAL READ PROTOCOL !!!
    // -----------------------------------------------------------------
    // Example: send a request byte (0x10) and expect for each motor:
    //   4 bytes position (float) + 4 bytes velocity (float)
    uint8_t request = 0x10;
    size_t written = port.serial->write(&request, 1);
    if (written != 1) {
      RCLCPP_WARN(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Failed to send read request on %s", port.device.c_str());
      continue;
    }

    size_t expected_bytes = port.motor_ids.size() * 8;  // 4 pos + 4 vel per motor
    std::vector<uint8_t> buffer(expected_bytes);
    size_t read_bytes = port.serial->read(buffer.data(), expected_bytes);
    if (read_bytes != expected_bytes) {
      RCLCPP_WARN(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Incomplete read on %s: got %zu, expected %zu",
        port.device.c_str(), read_bytes, expected_bytes);
      continue;
    }

    // Parse binary data (assume little‑endian float)
    for (size_t i = 0; i < port.motor_ids.size(); ++i) {
      float pos_f, vel_f;
      memcpy(&pos_f, &buffer[i * 8], 4);
      memcpy(&vel_f, &buffer[i * 8 + 4], 4);
      port.pos[i] = static_cast<double>(pos_f);
      port.vel[i] = static_cast<double>(vel_f);
    }
    // -----------------------------------------------------------------
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeliveryRobotHardware::write()
{
  for (auto & port : ports_) {
    // -----------------------------------------------------------------
    // !!! REPLACE THIS SECTION WITH YOUR ACTUAL WRITE PROTOCOL !!!
    // -----------------------------------------------------------------
    // Example: packet: command (0x20), motor count,
    // then for each motor: [motor_id (1 byte)] [velocity (4 bytes float)]
    std::vector<uint8_t> packet;
    packet.push_back(0x20);
    packet.push_back(static_cast<uint8_t>(port.motor_ids.size()));
    for (size_t i = 0; i < port.motor_ids.size(); ++i) {
      packet.push_back(static_cast<uint8_t>(port.motor_ids[i]));  // motor ID
      float vel = static_cast<float>(port.cmd_vel[i]);
      uint8_t * vel_bytes = reinterpret_cast<uint8_t *>(&vel);
      packet.insert(packet.end(), vel_bytes, vel_bytes + 4);
    }

    size_t written = port.serial->write(packet.data(), packet.size());
    if (written != packet.size()) {
      RCLCPP_WARN(
        rclcpp::get_logger("DeliveryRobotHardware"),
        "Failed to send full command on %s", port.device.c_str());
    }
    // -----------------------------------------------------------------
  }
  return hardware_interface::return_type::OK;
}

}  // namespace delivery_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(delivery_robot::DeliveryRobotHardware, hardware_interface::SystemInterface)