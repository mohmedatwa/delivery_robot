#include "delivery_firmware/delivery_firmware.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace delivery_firmware
{

hardware_interface::CallbackReturn DeliveryFirmware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info.joints.size();

  if (num_joints != 4)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DeliveryFirmware"),
                 "Expected 4 joints, got %ld", num_joints);
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_commands_.assign(num_joints, 0.0);

  
  device_ = info.hardware_parameters.at("device");
  baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger("DeliveryFirmware"),
              "DeliveryFirmware initialized");
  RCLCPP_INFO(rclcpp::get_logger("DeliveryFirmware"),
              "Device: %s | Baud: %d",
              device_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DeliveryFirmware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DeliveryFirmware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type DeliveryFirmware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  double dt = period.seconds();

  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] += hw_velocities_[i] * dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeliveryFirmware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_velocities_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace delivery_firmware

PLUGINLIB_EXPORT_CLASS(
  delivery_firmware::DeliveryFirmware,
  hardware_interface::SystemInterface)