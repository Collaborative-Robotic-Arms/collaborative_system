cmake_minimum_required(VERSION 3.8)
project(abb_gripper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(abb_robot_msgs REQUIRED)

add_library(abb_gripper_controller SHARED 
  src/abb_gripper_controller.cpp
)

target_include_directories(abb_gripper_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(abb_gripper_controller
  hardware_interface
  rclcpp
  abb_robot_msgs
  pluginlib
  rclcpp_lifecycle
)

# FIXED THIS LINE: Changed controller_interface to hardware_interface
pluginlib_export_plugin_description_file(hardware_interface abb_gripper_controller.xml)

# Add install rules so the plugin can be found after building
install(TARGETS abb_gripper_controller
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES abb_gripper_controller.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()