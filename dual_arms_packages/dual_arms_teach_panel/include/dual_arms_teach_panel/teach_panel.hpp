#pragma once

// ── RViz ──────────────────────────────────────────────────────
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>   // ← REQUIRED: fixes "incomplete type DisplayContext"

// ── ROS2 ──────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// ── MoveIt2 ───────────────────────────────────────────────────
// NOTE: .hpp not .h  — .h is obsolete in ROS2 Jazzy and causes
//       the "incomplete type DisplayContext" cascade error
#include <moveit/move_group_interface/move_group_interface.hpp>

// ── Qt5 ───────────────────────────────────────────────────────
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QListWidget>
#include <QTabWidget>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QTimer>

// ── STL ───────────────────────────────────────────────────────
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace dual_arms_teach_panel {

  // ─────────────────────────────────────────────────────────────
  // Waypoint: one named robot pose (stored in a program)
  // ─────────────────────────────────────────────────────────────
  struct Waypoint {
    std::string name;
    std::string command_type = "waypoint";  // "waypoint" | "gripper"
    std::string robot;           // "abb_irb120" | "ar4_mk3"
    std::vector<double> joints;  // joint positions in radians
    geometry_msgs::msg::Pose ee_pose;
    std::string gripper_state;   // "open" | "closed" for gripper commands
    bool validated = false;
    std::string timestamp;
  };

  // ─────────────────────────────────────────────────────────────
  // TeachPanel: main RViz2 panel plugin
  // ─────────────────────────────────────────────────────────────
  class TeachPanel : public rviz_common::Panel {
    Q_OBJECT

  public:
    explicit TeachPanel(QWidget* parent = nullptr);
    ~TeachPanel() override = default;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

  private slots:
    // Teach mode
    void onTeachModeChanged(int index);
    void onRobotChanged(int index);
    void onRecordWaypoint();
    void onRecordGripperCommand();
    void onGoToWaypoint();
    void onDeleteWaypoint();
    void onWaypointSelected(int row);

    // Manual XYZ
    void onSetFromXYZ();

    // Program management
    void onProgramSelected(int index);
    void onSaveProgram();
    void onLoadProgram();
    void onValidateInSim();
    void onExecuteReal();

    // Servo jog (called by jog_timer_ at 50 Hz)
    void onJogTick();
    // Called by ServoJogWidget::jogTwist — moves the interactive marker
    void onServoJogTwist(const geometry_msgs::msg::Twist& twist);

    // Joint state callback (marshalled to Qt thread)
    void onJointStatesReceived(const sensor_msgs::msg::JointState::SharedPtr msg);

  private:
    // ── UI builders ──────────────────────────────────────────────
    void buildUI();
    void buildMarkerDragTab(QWidget* tab);
    void buildServoJogTab(QWidget* tab);
    void buildManualXYZTab(QWidget* tab);

    // ── Helpers ──────────────────────────────────────────────────
    void updateCurrentPoseDisplay();
    void updateWaypointList();
    void refreshProgramList();
    std::string getCurrentTimestamp();
    bool planToWaypoint(const Waypoint& wp);
    std::chrono::steady_clock::time_point last_display_update_;
    
    // in teach_panel.hpp private:
    void initializeMoveItForRobot(const std::string& robot);
    void autoSelectRobotFromJointStates(const sensor_msgs::msg::JointState& msg);
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      getMoveGroupForRobot(const std::string& robot) const;
    bool isMoveGroupReady(const std::string& robot) const;
    bool robotVisibleInJointStates(const std::string& robot) const;
    std::string planningGroupForRobot(const std::string& robot) const;
    bool planningGroupAvailableForRobot(const std::string& robot) const;

    bool abb_moveit_initialized_ = false;
    bool ar4_moveit_initialized_ = false;
    bool abb_moveit_init_requested_ = false;
    bool ar4_moveit_init_requested_ = false;
    // ── ROS2 ─────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr program_exec_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr estop_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr move_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr program_validate_pub_;

    // Subscribes to the MotionPlanning display's interactive marker feedback.
    // Gives us the current marker EE pose goal without touching the state monitor.
    rclcpp::Subscription<
      visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_feedback_sub_;

    // Latest EE pose received from interactive marker drag.
    geometry_msgs::msg::Pose latest_marker_pose_;
    bool marker_pose_received_ = false;

    // Accumulated EE pose for servo jog (updated incrementally by onServoJogTwist).
    geometry_msgs::msg::Pose servo_target_pose_;
    bool servo_initialized_ = false;

    // EE pose set by Manual XYZ spinboxes (onSetFromXYZ).
    geometry_msgs::msg::Pose manual_target_pose_;

    // Servo jog widget (created in onInitialize after node_ is ready)
    class ServoJogWidget* servo_jog_widget_ = nullptr;
    QWidget* servo_tab_widget_ = nullptr;

    // ── MoveIt2 ──────────────────────────────────────────────────
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> abb_mg_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ar4_mg_;

    // ── State ────────────────────────────────────────────────────
    sensor_msgs::msg::JointState::SharedPtr current_joint_states_;
    geometry_msgs::msg::Pose current_ee_pose_;
    std::vector<Waypoint> waypoints_;
    std::string active_robot_ = "abb_irb120";
    std::string active_program_ = "program_001";
    int jog_mode_ = 0;  // 0=marker, 1=servo, 2=manual

    // Jog velocities fed to moveit_servo (servo mode only)
    geometry_msgs::msg::Twist jog_cmd_;
    QTimer* jog_timer_;

    // ── UI widgets ───────────────────────────────────────────────
    QComboBox* robot_selector_;
    QTabWidget* teach_tabs_;
    QLabel* pose_display_;
    QLabel* joint_display_;
    QLineEdit* waypoint_name_edit_;
    QComboBox* gripper_state_selector_;
    QListWidget* waypoint_list_;
    QLineEdit* program_name_edit_;
    QComboBox* program_selector_;
    QLabel* status_label_;

    // Manual XYZ spinboxes
    QDoubleSpinBox* x_spin_;
    QDoubleSpinBox* y_spin_;
    QDoubleSpinBox* z_spin_;
    QDoubleSpinBox* rx_spin_;
    QDoubleSpinBox* ry_spin_;
    QDoubleSpinBox* rz_spin_;
  };

}  // namespace dual_arms_teach_panel
