cmake_minimum_required(VERSION 3.8)
project(dual_arms_teach_panel)

# 1. Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. Handle Qt MOC
set(MOC_HEADERS
  include/dual_arms_teach_panel/teach_panel.hpp
  include/dual_arms_teach_panel/servo_jog_widget.hpp
  include/dual_arms_teach_panel/waypoint_list_widget.hpp
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})
set(CMAKE_AUTOMOC OFF)

# 3. DEFINE THE TARGET FIRST (Moved this up)
add_library(dual_arms_teach_panel SHARED
  src/teach_panel.cpp
  src/waypoint_list_widget.cpp
  src/servo_jog_widget.cpp
  src/program_manager.cpp
  ${MOC_SOURCES}
)

# 4. Now you can apply settings to the target
target_include_directories(dual_arms_teach_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(dual_arms_teach_panel 
  Eigen3::Eigen
  Qt5::Widgets
  Qt5::Core
  yaml-cpp
)

ament_target_dependencies(dual_arms_teach_panel
  rclcpp
  rviz_common
  rviz_default_plugins
  pluginlib
  moveit_ros_planning_interface
  moveit_servo
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

# 5. Export and Install
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS dual_arms_teach_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()