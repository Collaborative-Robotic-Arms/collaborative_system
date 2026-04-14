#ifndef ABB_GRIPPER_DRIVER__ABB_GRIPPER_HW_INTERFACE_HPP_
#define ABB_GRIPPER_DRIVER__ABB_GRIPPER_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "abb_robot_msgs/srv/set_rapid_bool.hpp"

namespace abb_gripper_driver
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ABBGripperHWInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Internal function to trigger ABB RWS
  void call_gripper_service(bool open);

  // ROS 2 communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr real_gripper_client_;

  // Parameters and state
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  bool last_state_was_open_;
};

}  // namespace abb_gripper_driver

#endif  // ABB_GRIPPER_DRIVER__ABB_GRIPPER_HW_INTERFACE_HPP_