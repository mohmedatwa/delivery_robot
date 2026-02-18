#ifndef DELIVERY_ROBOT__DELIVERY_ROBOT_HARDWARE_HPP_
#define DELIVERY_ROBOT__DELIVERY_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

namespace delivery_robot
{

/// Holds data and connection for one serial port
struct MotorPort
{
  std::string device;                     // e.g., /dev/ttyUSB0
  int baudrate;                            // e.g., 115200
  std::vector<std::string> joint_names;    // Joints attached to this port
  std::vector<int> motor_ids;               // Local motor IDs (as understood by hardware)
  std::shared_ptr<serial::Serial> serial;  // Serial connection

  // Storage for this port's joints
  std::vector<double> cmd_vel;              // commanded velocities
  std::vector<double> pos;                   // current positions
  std::vector<double> vel;                   // current velocities
};

class DeliveryRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DeliveryRobotHardware);

  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  // List of serial ports (each with its own set of joints)
  std::vector<MotorPort> ports_;

  // Mapping from global joint index (as in info_.joints) to (port_index, local_index)
  std::vector<std::pair<size_t, size_t>> joint_to_port_;
};

}  // namespace delivery_robot

#endif  // DELIVERY_ROBOT__DELIVERY_ROBOT_HARDWARE_HPP_