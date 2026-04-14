#include "abb_gripper_controller/abb_gripper_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "abb_robot_msgs/msg/service_responses.hpp"

namespace abb_gripper_driver
{

CallbackReturn ABBGripperHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("abb_gripper_hw_interface_node");
  
  // Connect to the ABB RWS Service
  real_gripper_client_ = node_->create_client<abb_robot_msgs::srv::SetRAPIDBool>(
    "/rws_client/set_gripper_state"
  );

  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_.resize(info_.joints.size(), 0.0);
  last_state_was_open_ = false;

  RCLCPP_INFO(node_->get_logger(), "ABB Gripper HW Initialized for %zu joints", info_.joints.size());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ABBGripperHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBGripperHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ABBGripperHWInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    hw_states_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ABBGripperHWInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool should_be_open = (hw_commands_[0] > 0.01);

  if (should_be_open != last_state_was_open_)
  {
    if (real_gripper_client_->service_is_ready())
    {
      call_gripper_service(should_be_open);
      RCLCPP_INFO(node_->get_logger(), "Requesting Gripper: %s", should_be_open ? "OPEN" : "CLOSE");
      last_state_was_open_ = should_be_open;
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ABB Gripper Service NOT ready!");
    }
  }
  return hardware_interface::return_type::OK;
}

void ABBGripperHWInterface::call_gripper_service(bool open)
{
  auto request = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
  request->path.task = "T_ROB1";
  request->path.module = "egm"; // Check that this is the correct module on your IRC5/Omnicore
  request->path.symbol = "gripper_close"; 
  request->value = open; 

  // Async call: Does not block the real-time loop.
  // We do not wait for the response here to keep the controller cycle fast.
  real_gripper_client_->async_send_request(request);
}

CallbackReturn ABBGripperHWInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating Gripper Hardware...");
  return CallbackReturn::SUCCESS;
}

}  // namespace abb_gripper_driver

PLUGINLIB_EXPORT_CLASS(
  abb_gripper_driver::ABBGripperHWInterface, 
  hardware_interface::SystemInterface
)