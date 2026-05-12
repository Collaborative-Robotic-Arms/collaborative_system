#include "dual_arms_teach_panel/teach_panel.hpp"
#include "dual_arms_teach_panel/program_manager.hpp"
#include "dual_arms_teach_panel/servo_jog_widget.hpp"
#include "dual_arms_teach_panel/waypoint_list_widget.hpp"

#include <QMessageBox>
#include <QGroupBox>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSignalBlocker>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <thread>
#include <cmath>
#include <cctype>

namespace dual_arms_teach_panel {

  // ─────────────────────────────────────────────────────────────
  // Constructor
  // ─────────────────────────────────────────────────────────────
  TeachPanel::TeachPanel(QWidget* parent)
    : rviz_common::Panel(parent)
  {
    buildUI();

    jog_timer_ = new QTimer(this);
    jog_timer_->setInterval(20);  // 50 Hz
    connect(jog_timer_, &QTimer::timeout, this, &TeachPanel::onJogTick);
  }

  // ─────────────────────────────────────────────────────────────
  // onInitialize — called by RViz after the display context is ready
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::onInitialize()
  {
    auto* context = getDisplayContext();
    if (!context) {
      status_label_->setText("RViz display context is not ready.");
      return;
    }

    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (!ros_node_abstraction) {
      status_label_->setText("Failed to acquire RViz ROS node.");
      return;
    }

    node_ = ros_node_abstraction->get_raw_node();
    if (!node_) {
      status_label_->setText("RViz ROS node is null.");
      return;
    }

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joint_states_ = msg;

        autoSelectRobotFromJointStates(*msg);

        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_display_update_).count();
        if (dt < 100) {
          return;
        }
        last_display_update_ = now;

        if (!isMoveGroupReady(active_robot_)) {
          initializeMoveItForRobot(active_robot_);
          return;
        }

        QMetaObject::invokeMethod(
          this, &TeachPanel::updateCurrentPoseDisplay, Qt::QueuedConnection);
      });

    program_validate_pub_ =
      node_->create_publisher<std_msgs::msg::String>("/dt/program_validate", 1);

    program_exec_pub_ =
      node_->create_publisher<std_msgs::msg::String>("/dt/program_execute", 1);

    estop_pub_ =
      node_->create_publisher<std_msgs::msg::String>("/dt/estop", 1);

    move_marker_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/rviz/moveit/move_marker/goal", 10);

    marker_feedback_sub_ =
      node_->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        "/rviz_moveit_motion_planning_display"
        "/robot_interaction_interactive_marker_topic/feedback",
        10,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
          if (msg->event_type !=
            visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            return;
          }

          latest_marker_pose_ = msg->pose;
          marker_pose_received_ = true;

          if (!servo_initialized_) {
            servo_target_pose_ = msg->pose;
          }
        });

    if (servo_tab_widget_) {
      servo_jog_widget_ = new ServoJogWidget(node_, servo_tab_widget_);
      auto* tab_layout = qobject_cast<QVBoxLayout*>(servo_tab_widget_->layout());
      if (tab_layout) {
        QLayoutItem* item = tab_layout->takeAt(0);
        if (item) {
          delete item->widget();
          delete item;
        }
        tab_layout->insertWidget(0, servo_jog_widget_);
      }

      connect(servo_jog_widget_, &ServoJogWidget::recordRequested,
        this, &TeachPanel::onRecordWaypoint);
      connect(servo_jog_widget_, &ServoJogWidget::jogTwist,
        this, &TeachPanel::onServoJogTwist);
    }

    status_label_->setText("Waiting for joint states...");
  }


  void TeachPanel::initializeMoveItForRobot(const std::string& robot)
  {
    if (!node_) {
      status_label_->setText("ROS node is not ready yet.");
      return;
    }

    if (!planningGroupAvailableForRobot(robot)) {
      if (robot == "abb_irb120" && planningGroupAvailableForRobot("ar4_mk3")) {
        active_robot_ = "ar4_mk3";
        int idx = robot_selector_->findText("ar4_mk3");
        if (idx >= 0) {
          QMetaObject::invokeMethod(robot_selector_, [this, idx]() {
            robot_selector_->setCurrentIndex(idx);
          }, Qt::QueuedConnection);
        }
        QMetaObject::invokeMethod(this, [this]() {
          status_label_->setText(
            "ABB planning group is unavailable in this RViz session. Switched to AR4.");
        }, Qt::QueuedConnection);
      }
      else {
        status_label_->setText(
          QString("MoveIt group '%1' is not available in this session.")
          .arg(planningGroupForRobot(robot).c_str()));
      }
      return;
    }

    if (!robotVisibleInJointStates(robot)) {
      status_label_->setText(
        QString("%1 is not present in the current /joint_states stream.")
        .arg(robot.c_str()));
      return;
    }

    if (robot == "abb_irb120") {
      if (abb_moveit_initialized_ || abb_moveit_init_requested_) {
        return;
      }
      abb_moveit_init_requested_ = true;
    }
    else if (robot == "ar4_mk3") {
      if (ar4_moveit_initialized_ || ar4_moveit_init_requested_) {
        return;
      }
      ar4_moveit_init_requested_ = true;
    }
    else {
      status_label_->setText(QString("Unknown robot: %1").arg(robot.c_str()));
      return;
    }

    status_label_->setText(
      QString("Initializing MoveIt for %1...").arg(robot.c_str()));

    std::thread([this, robot]() {
      try {
        moveit::planning_interface::MoveGroupInterface::Options opts(
          planningGroupForRobot(robot));

        auto move_group =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, opts);

        QMetaObject::invokeMethod(this, [this, robot, move_group]() {
          if (robot == "abb_irb120") {
            abb_mg_ = move_group;
            abb_moveit_initialized_ = true;
            abb_moveit_init_requested_ = false;
          }
          else {
            ar4_mg_ = move_group;
            ar4_moveit_initialized_ = true;
            ar4_moveit_init_requested_ = false;
          }

          status_label_->setText(
            QString("MoveIt ready for %1.").arg(robot.c_str()));
          }, Qt::QueuedConnection);

      }
      catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, robot, message = std::string(e.what())]() {
          if (robot == "abb_irb120") {
            abb_moveit_init_requested_ = false;
          }
          else {
            ar4_moveit_init_requested_ = false;
          }

          status_label_->setText(
            QString("MoveIt init failed for %1: %2")
            .arg(robot.c_str())
            .arg(message.c_str()));
          }, Qt::QueuedConnection);
      }
      }).detach();
  }

  void TeachPanel::autoSelectRobotFromJointStates(
    const sensor_msgs::msg::JointState& msg)
  {
    bool has_abb = false;
    bool has_ar4 = false;

    for (const auto& name : msg.name) {
      if (name.rfind("joint_", 0) == 0) {
        has_abb = true;
      }
      if (name.rfind("ar4_joint_", 0) == 0) {
        has_ar4 = true;
      }
    }

    if (has_ar4 && !planningGroupAvailableForRobot(active_robot_) &&
      planningGroupAvailableForRobot("ar4_mk3") && active_robot_ != "ar4_mk3") {
      active_robot_ = "ar4_mk3";
      int idx = robot_selector_->findText("ar4_mk3");
      if (idx >= 0) {
        robot_selector_->setCurrentIndex(idx);
      }
    }
    else if (has_ar4 && !has_abb && active_robot_ != "ar4_mk3") {
      active_robot_ = "ar4_mk3";
      int idx = robot_selector_->findText("ar4_mk3");
      if (idx >= 0) {
        robot_selector_->setCurrentIndex(idx);
      }
    }
    else if (has_abb && !has_ar4 && active_robot_ != "abb_irb120") {
      active_robot_ = "abb_irb120";
      int idx = robot_selector_->findText("abb_irb120");
      if (idx >= 0) {
        robot_selector_->setCurrentIndex(idx);
      }
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
    TeachPanel::getMoveGroupForRobot(const std::string& robot) const
  {
    if (robot == "abb_irb120") {
      return abb_mg_;
    }
    if (robot == "ar4_mk3") {
      return ar4_mg_;
    }
    return nullptr;
  }

  bool TeachPanel::isMoveGroupReady(const std::string& robot) const
  {
    return static_cast<bool>(getMoveGroupForRobot(robot));
  }

  bool TeachPanel::robotVisibleInJointStates(const std::string& robot) const
  {
    if (!current_joint_states_) {
      return true;
    }

    const std::string prefix =
      (robot == "abb_irb120") ? "joint_" : "ar4_joint_";

    return std::any_of(
      current_joint_states_->name.begin(),
      current_joint_states_->name.end(),
      [&prefix](const std::string& name) {
        return name.rfind(prefix, 0) == 0;
      });
  }

  std::string TeachPanel::planningGroupForRobot(const std::string& robot) const
  {
    if (robot == "abb_irb120") {
      return "irb120_arm";
    }
    if (robot == "ar4_mk3") {
      return "ar_manipulator";
    }
    return "";
  }

  bool TeachPanel::planningGroupAvailableForRobot(const std::string& robot) const
  {
    if (!node_) {
      return false;
    }

    const std::string group = planningGroupForRobot(robot);
    if (group.empty()) {
      return false;
    }

    std::string semantic_description;
    if (!node_->get_parameter("robot_description_semantic", semantic_description)) {
      return true;
    }

    return semantic_description.find("group name=\"" + group + "\"") != std::string::npos;
  }



  // ─────────────────────────────────────────────────────────────
  // Timestamp helper
  // ─────────────────────────────────────────────────────────────
  std::string TeachPanel::getCurrentTimestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%dT%H:%M:%S");
    return ss.str();
  }

  // ─────────────────────────────────────────────────────────────
  // buildUI — constructs the full panel widget tree
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::buildUI()
  {
    auto* root_layout = new QVBoxLayout(this);
    root_layout->setContentsMargins(6, 6, 6, 6);
    root_layout->setSpacing(6);

    // ── Header ────────────────────────────────────────────────
    auto* header = new QLabel("<b>⬡ Teach &amp; Record Panel</b>");
    header->setAlignment(Qt::AlignCenter);
    header->setStyleSheet("background:#1A3A5C; color:white; padding:6px; border-radius:4px;");
    root_layout->addWidget(header);

    // ── Robot selector ────────────────────────────────────────
    auto* robot_row = new QHBoxLayout();
    robot_row->addWidget(new QLabel("Robot:"));
    robot_selector_ = new QComboBox();
    robot_selector_->addItems({ "abb_irb120", "ar4_mk3" });
    connect(robot_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
      this, &TeachPanel::onRobotChanged);
    robot_row->addWidget(robot_selector_);
    root_layout->addLayout(robot_row);

    // ── Current pose display ──────────────────────────────────
    auto* pose_group = new QGroupBox("Current Pose");
    auto* pose_layout = new QVBoxLayout(pose_group);
    pose_display_ = new QLabel("EE:  waiting...");
    joint_display_ = new QLabel("J:   waiting...");
    pose_display_->setFont(QFont("Courier", 9));
    joint_display_->setFont(QFont("Courier", 9));
    pose_layout->addWidget(pose_display_);
    pose_layout->addWidget(joint_display_);
    root_layout->addWidget(pose_group);

    // ── Teach mode tabs ───────────────────────────────────────
    teach_tabs_ = new QTabWidget();
    auto* marker_tab = new QWidget();
    auto* servo_tab = new QWidget();
    auto* manual_tab = new QWidget();
    buildMarkerDragTab(marker_tab);
    buildServoJogTab(servo_tab);
    buildManualXYZTab(manual_tab);
    teach_tabs_->addTab(marker_tab, "Marker Drag");
    teach_tabs_->addTab(servo_tab, "Servo Jog");
    teach_tabs_->addTab(manual_tab, "Manual XYZ");
    servo_tab_widget_ = servo_tab;  // saved so onInitialize() can inject ServoJogWidget
    connect(teach_tabs_, &QTabWidget::currentChanged,
      this, &TeachPanel::onTeachModeChanged);
    root_layout->addWidget(teach_tabs_);

    // ── Record waypoint ───────────────────────────────────────
    auto* record_group = new QGroupBox("Record Waypoint");
    auto* record_layout = new QVBoxLayout(record_group);
    auto* name_row = new QHBoxLayout();
    name_row->addWidget(new QLabel("Name:"));
    waypoint_name_edit_ = new QLineEdit();
    waypoint_name_edit_->setPlaceholderText("auto if empty");
    name_row->addWidget(waypoint_name_edit_);
    record_layout->addLayout(name_row);
    auto* record_btn = new QPushButton("⬤  RECORD WAYPOINT");
    record_btn->setStyleSheet(
      "background:#166534; color:white; font-weight:bold; padding:8px; border-radius:4px;");
    connect(record_btn, &QPushButton::clicked, this, &TeachPanel::onRecordWaypoint);
    record_layout->addWidget(record_btn);

    auto* gripper_row = new QHBoxLayout();
    gripper_row->addWidget(new QLabel("Gripper:"));
    gripper_state_selector_ = new QComboBox();
    gripper_state_selector_->addItems({ "open", "closed" });
    gripper_row->addWidget(gripper_state_selector_);
    auto* gripper_btn = new QPushButton("Record Gripper");
    connect(gripper_btn, &QPushButton::clicked,
      this, &TeachPanel::onRecordGripperCommand);
    gripper_row->addWidget(gripper_btn);
    record_layout->addLayout(gripper_row);
    root_layout->addWidget(record_group);

    // ── Waypoint list ─────────────────────────────────────────
    auto* list_group = new QGroupBox("Recorded Waypoints");
    auto* list_layout = new QVBoxLayout(list_group);
    waypoint_list_ = new QListWidget();
    waypoint_list_->setMaximumHeight(140);
    connect(waypoint_list_, &QListWidget::currentRowChanged,
      this, &TeachPanel::onWaypointSelected);
    list_layout->addWidget(waypoint_list_);

    auto* wp_btn_row = new QHBoxLayout();
    auto* goto_btn = new QPushButton("▶ Go To");
    auto* delete_btn = new QPushButton("✗ Delete");
    connect(goto_btn, &QPushButton::clicked, this, &TeachPanel::onGoToWaypoint);
    connect(delete_btn, &QPushButton::clicked, this, &TeachPanel::onDeleteWaypoint);
    wp_btn_row->addWidget(goto_btn);
    wp_btn_row->addWidget(delete_btn);
    list_layout->addLayout(wp_btn_row);
    root_layout->addWidget(list_group);

    // ── Program management ────────────────────────────────────
    auto* prog_group = new QGroupBox("Program");
    auto* prog_layout = new QVBoxLayout(prog_group);
    auto* prog_name_row = new QHBoxLayout();
    prog_name_row->addWidget(new QLabel("Name:"));
    program_name_edit_ = new QLineEdit("program_001");
    prog_name_row->addWidget(program_name_edit_);
    prog_layout->addLayout(prog_name_row);

    auto* prog_select_row = new QHBoxLayout();
    prog_select_row->addWidget(new QLabel("Saved:"));
    program_selector_ = new QComboBox();
    program_selector_->setMinimumContentsLength(18);
    program_selector_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    connect(program_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
      this, &TeachPanel::onProgramSelected);
    prog_select_row->addWidget(program_selector_);
    prog_layout->addLayout(prog_select_row);
    refreshProgramList();

    auto* prog_btn_row = new QHBoxLayout();
    auto* save_btn = new QPushButton("💾 Save YAML");
    auto* load_btn = new QPushButton("📂 Load");
    auto* validate_btn = new QPushButton("▶ Validate in Sim");
    auto* execute_btn = new QPushButton("⚡ Execute Real");

    save_btn->setStyleSheet("background:#1D4ED8; color:white; border-radius:3px; padding:5px;");
    validate_btn->setStyleSheet("background:#6D28D9; color:white; border-radius:3px; padding:5px;");
    execute_btn->setStyleSheet("background:#991B1B; color:white; border-radius:3px; padding:5px;");

    connect(save_btn, &QPushButton::clicked, this, &TeachPanel::onSaveProgram);
    connect(load_btn, &QPushButton::clicked, this, &TeachPanel::onLoadProgram);
    connect(validate_btn, &QPushButton::clicked, this, &TeachPanel::onValidateInSim);
    connect(execute_btn, &QPushButton::clicked, this, &TeachPanel::onExecuteReal);

    prog_btn_row->addWidget(save_btn);
    prog_btn_row->addWidget(load_btn);
    prog_layout->addLayout(prog_btn_row);
    prog_layout->addWidget(validate_btn);
    prog_layout->addWidget(execute_btn);
    root_layout->addWidget(prog_group);

    // ── Status bar ────────────────────────────────────────────
    status_label_ = new QLabel("Initializing...");
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet("background:#F1F5F9; padding:4px; border-radius:3px; font-size:10px;");
    root_layout->addWidget(status_label_);

    setLayout(root_layout);
  }

  void TeachPanel::buildMarkerDragTab(QWidget* tab)
  {
    auto* layout = new QVBoxLayout(tab);
    layout->addWidget(new QLabel(
      "Drag the interactive marker in the 3D view\n"
      "to position the end-effector, then press\n"
      "RECORD WAYPOINT below."));
    layout->addStretch();
  }

  void TeachPanel::buildServoJogTab(QWidget* tab)
  {
    auto* layout = new QVBoxLayout(tab);
    // ServoJogWidget is constructed after node_ is available (in onInitialize),
    // so we add a placeholder label here; the actual widget is added in onInitialize.
    layout->addWidget(new QLabel(
      "Keyboard jog (focus this tab first):\n"
      "  W/S  — X+/-    A/D  — Y+/-\n"
      "  R/F  — Z+/-    Q/E  — Rz+/-\n"
      "  Z/X  — Rx+/-   SPACE — Record\n\n"
      "Servo node must be running."));
    layout->addStretch();
  }

  void TeachPanel::buildManualXYZTab(QWidget* tab)
  {
    auto* layout = new QGridLayout(tab);
    layout->addWidget(new QLabel("X (m):"), 0, 0);  x_spin_ = new QDoubleSpinBox(); x_spin_->setRange(-2.0, 2.0); x_spin_->setSingleStep(0.01); layout->addWidget(x_spin_, 0, 1);
    layout->addWidget(new QLabel("Y (m):"), 1, 0);  y_spin_ = new QDoubleSpinBox(); y_spin_->setRange(-2.0, 2.0); y_spin_->setSingleStep(0.01); layout->addWidget(y_spin_, 1, 1);
    layout->addWidget(new QLabel("Z (m):"), 2, 0);  z_spin_ = new QDoubleSpinBox(); z_spin_->setRange(-2.0, 2.0); z_spin_->setSingleStep(0.01); layout->addWidget(z_spin_, 2, 1);
    layout->addWidget(new QLabel("Rx (°):"), 3, 0);  rx_spin_ = new QDoubleSpinBox(); rx_spin_->setRange(-180, 180); layout->addWidget(rx_spin_, 3, 1);
    layout->addWidget(new QLabel("Ry (°):"), 4, 0);  ry_spin_ = new QDoubleSpinBox(); ry_spin_->setRange(-180, 180); layout->addWidget(ry_spin_, 4, 1);
    layout->addWidget(new QLabel("Rz (°):"), 5, 0);  rz_spin_ = new QDoubleSpinBox(); rz_spin_->setRange(-180, 180); layout->addWidget(rz_spin_, 5, 1);

    auto* set_btn = new QPushButton("Set Target Pose");
    connect(set_btn, &QPushButton::clicked, this, &TeachPanel::onSetFromXYZ);
    layout->addWidget(set_btn, 6, 0, 1, 2);
  }

  // ─────────────────────────────────────────────────────────────
  // Slot implementations
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::onRobotChanged(int index)
  {
    active_robot_ = robot_selector_->itemText(index).toStdString();
    status_label_->setText(QString("Active robot: %1").arg(active_robot_.c_str()));

    marker_pose_received_ = false;
    servo_target_pose_ = geometry_msgs::msg::Pose();
    servo_target_pose_.orientation.w = 1.0;
    servo_initialized_ = false;

    if (!isMoveGroupReady(active_robot_)) {
      initializeMoveItForRobot(active_robot_);
    }
  }


  void TeachPanel::onTeachModeChanged(int index)
  {
    jog_mode_ = index;
    if (jog_mode_ == 1) {
      // Entering Servo Jog tab — reset so servo_target_pose_ re-seeds
      // from the current marker position (picked up via feedback subscriber).
      servo_initialized_ = false;
      jog_timer_->start();
    }
    else {
      jog_timer_->stop();
      jog_cmd_ = geometry_msgs::msg::Twist();
    }
  }

  void TeachPanel::onJogTick()
  {
    // Called at 50 Hz while servo jog tab is active.
    // Actual publishing is done by ServoJogWidget via keyPress/keyRelease.
  }

  // ─────────────────────────────────────────────────────────────
  // onServoJogTwist — move the interactive marker by the twist delta
  //
  // Called by ServoJogWidget::jogTwist signal on every key press/release.
  // Instead of executing on the robot, we offset the current marker pose
  // and republish it to /rviz/moveit/move_marker/goal_<group>.
  // The MotionPlanning display picks that up and moves the marker,
  // which then fires a POSE_UPDATE feedback → latest_marker_pose_ updated.
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::onServoJogTwist(const geometry_msgs::msg::Twist& twist)
  {
    // Initialise servo_target_pose_ from the current marker position
    // the first time a key is pressed (or after a robot switch).
    if (!servo_initialized_) {
      if (marker_pose_received_) {
        servo_target_pose_ = latest_marker_pose_;
      }
      else if (current_joint_states_) {
        // Fallback: use actual robot position as starting point
        servo_target_pose_ = geometry_msgs::msg::Pose();
        servo_target_pose_.position.x = 0.3;
        servo_target_pose_.position.z = 0.3;
        servo_target_pose_.orientation.w = 1.0;
      }
      else {
        return;  // nothing to start from
      }
      servo_initialized_ = true;
    }

    const double dt = 0.02;  // 50 Hz step size

    // ── Linear delta ───────────────────────────────────────────
    servo_target_pose_.position.x += twist.linear.x * dt;
    servo_target_pose_.position.y += twist.linear.y * dt;
    servo_target_pose_.position.z += twist.linear.z * dt;

    // ── Rotational delta (yaw = angular.z, roll = angular.x) ──
    // Compose quaternion with small rotation increments.
    // q_new = q_delta * q_old
    auto compose_yaw = [](geometry_msgs::msg::Pose& pose, double dAngle) {
      double c = std::cos(dAngle * 0.5), s = std::sin(dAngle * 0.5);
      double w = pose.orientation.w, z = pose.orientation.z;
      double x = pose.orientation.x, y = pose.orientation.y;
      // Rotate by dAngle around world Z
      pose.orientation.w = c * w - s * z;
      pose.orientation.x = c * x - s * y;
      pose.orientation.y = c * y + s * x;
      pose.orientation.z = c * z + s * w;
      // Re-normalize
      double n = std::sqrt(pose.orientation.w * pose.orientation.w +
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z);
      if (n > 1e-6) {
        pose.orientation.w /= n; pose.orientation.x /= n;
        pose.orientation.y /= n; pose.orientation.z /= n;
      }
      };

    auto compose_roll = [](geometry_msgs::msg::Pose& pose, double dAngle) {
      double c = std::cos(dAngle * 0.5), s = std::sin(dAngle * 0.5);
      double w = pose.orientation.w, x = pose.orientation.x;
      double y = pose.orientation.y, z = pose.orientation.z;
      pose.orientation.w = c * w - s * x;
      pose.orientation.x = c * x + s * w;
      pose.orientation.y = c * y + s * z;
      pose.orientation.z = c * z - s * y;
      double n = std::sqrt(pose.orientation.w * pose.orientation.w +
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z);
      if (n > 1e-6) {
        pose.orientation.w /= n; pose.orientation.x /= n;
        pose.orientation.y /= n; pose.orientation.z /= n;
      }
      };

    if (std::abs(twist.angular.z) > 1e-6)
      compose_yaw(servo_target_pose_, twist.angular.z * dt);
    if (std::abs(twist.angular.x) > 1e-6)
      compose_roll(servo_target_pose_, twist.angular.x * dt);

    // ── Move the RViz interactive marker to the new pose ─────────
    // Publishing to this topic makes the MotionPlanning display update
    // the 3D marker position so the user gets visual feedback while jogging.
    if (move_marker_pub_ && node_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = node_->now();
      ps.header.frame_id = (active_robot_ == "abb_irb120") ? "base_link" : "ar4_base_link";
      ps.pose = servo_target_pose_;
      move_marker_pub_->publish(ps);
    }

    // Update status bar for continuous visual feedback while jogging
    status_label_->setText(
      QString("Jog: X=%1 Y=%2 Z=%3")
      .arg(servo_target_pose_.position.x, 0, 'f', 3)
      .arg(servo_target_pose_.position.y, 0, 'f', 3)
      .arg(servo_target_pose_.position.z, 0, 'f', 3));
  }

  void TeachPanel::onSetFromXYZ()
  {
    if (!node_) {
      status_label_->setText("⚠ Not initialized yet.");
      return;
    }

    // Build target pose from spinboxes
    manual_target_pose_.position.x = x_spin_->value();
    manual_target_pose_.position.y = y_spin_->value();
    manual_target_pose_.position.z = z_spin_->value();

    double rx = rx_spin_->value() * M_PI / 180.0;
    double ry = ry_spin_->value() * M_PI / 180.0;
    double rz = rz_spin_->value() * M_PI / 180.0;

    double cy = std::cos(rz * 0.5), sy = std::sin(rz * 0.5);
    double cp = std::cos(ry * 0.5), sp = std::sin(ry * 0.5);
    double cr = std::cos(rx * 0.5), sr = std::sin(rx * 0.5);

    manual_target_pose_.orientation.w = cr * cp * cy + sr * sp * sy;
    manual_target_pose_.orientation.x = sr * cp * cy - cr * sp * sy;
    manual_target_pose_.orientation.y = cr * sp * cy + sr * cp * sy;
    manual_target_pose_.orientation.z = cr * cp * sy - sr * sp * cy;

    // Publish to move the RViz interactive marker visually
    if (move_marker_pub_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = node_->now();
      ps.header.frame_id = (active_robot_ == "abb_irb120") ? "base_link" : "ar4_base_link";
      ps.pose = manual_target_pose_;
      move_marker_pub_->publish(ps);
    }

    status_label_->setText(
      QString("📍 Target: X=%1 Y=%2 Z=%3 — press RECORD to save.")
      .arg(manual_target_pose_.position.x, 0, 'f', 3)
      .arg(manual_target_pose_.position.y, 0, 'f', 3)
      .arg(manual_target_pose_.position.z, 0, 'f', 3));
  }

  void TeachPanel::onRecordWaypoint()
  {
    // ── Guard: MoveIt2 must be fully initialized ─────────────────
    if (!isMoveGroupReady(active_robot_)) {
      initializeMoveItForRobot(active_robot_);
      status_label_->setText(
        QString("MoveIt for %1 is not ready yet — wait a moment and try again.")
        .arg(active_robot_.c_str()));
      return;
    }


    // ── Determine pose source based on teach mode ─────────────────
    //
    //  Mode 0 (Marker Drag): use latest_marker_pose_ from feedback subscriber.
    //    The user dragged the RViz interactive marker. Record the EE goal pose.
    //
    //  Mode 1 (Servo Jog): use servo_target_pose_ accumulated by onServoJogTwist.
    //    The user jogged the EE incrementally. Record that accumulated pose.
    //
    //  Mode 2 (Manual XYZ): use manual_target_pose_ set by onSetFromXYZ.
    //    The user typed in XYZ/RPY values. Record that pose.
    //
    // In all cases: pose → IK → joint vector. No state monitor, no planning.

    geometry_msgs::msg::Pose target_pose;
    bool have_pose = false;

    if (jog_mode_ == 0) {
      if (!marker_pose_received_) {
        status_label_->setText(
          "⚠ Drag the interactive marker first, then press Record.");
        return;
      }
      target_pose = latest_marker_pose_;
      have_pose = true;
    }
    else if (jog_mode_ == 1) {
      target_pose = servo_target_pose_;
      have_pose = true;
    }
    else if (jog_mode_ == 2) {
      target_pose = manual_target_pose_;
      have_pose = true;
    }

    if (!have_pose) {
      status_label_->setText("⚠ No target pose set.");
      return;
    }

    // Snapshot waypoint fields now (on Qt thread, safe)
    std::string wp_name = waypoint_name_edit_->text().toStdString();
    std::string wp_robot = active_robot_;
    std::string wp_ts = getCurrentTimestamp();
    int         wp_idx = static_cast<int>(waypoints_.size()) + 1;

    status_label_->setText("⏳ Solving IK...");

    // ── Run IK on a background thread ────────────────────────────
    // setApproximateJointValueTarget internally calls IK. We must NOT
    // call it on the Qt main thread — it can block and cause the segfault.
    std::thread([this, target_pose, wp_name, wp_robot, wp_ts, wp_idx]() {
      auto mg = getMoveGroupForRobot(wp_robot);
      if (!mg) {
        QMetaObject::invokeMethod(this, [this, wp_robot]() {
          status_label_->setText(
            QString("No MoveIt group available for %1.").arg(wp_robot.c_str()));
          }, Qt::QueuedConnection);
        return;
      }

      // setApproximateJointValueTarget runs the IK solver and stores
      // the result as the goal joint state inside the MoveGroupInterface.
      mg->setApproximateJointValueTarget(target_pose, mg->getEndEffectorLink());

      // Read back the IK result — this is just a vector copy, no ROS call
      std::vector<double> joints;
      mg->getJointValueTarget(joints);

      // Marshal back to Qt thread to update UI and waypoints_
      QMetaObject::invokeMethod(this, [this, joints, target_pose,
        wp_name, wp_robot, wp_ts, wp_idx]() {
          if (joints.empty()) {
            status_label_->setText(
              "❌ IK failed for this pose — try a different position.");
            return;
          }

          Waypoint wp;
          wp.command_type = "waypoint";
          wp.joints = joints;
          wp.ee_pose = target_pose;
          wp.robot = wp_robot;
          wp.gripper_state.clear();
          wp.timestamp = wp_ts;
          wp.validated = false;
          wp.name = wp_name.empty()
            ? wp_robot.substr(0, 3) + "_wp_" + std::to_string(wp_idx)
            : wp_name;

          waypoint_name_edit_->setText(QString::fromStdString(wp.name));
          waypoints_.push_back(wp);
          updateWaypointList();

          status_label_->setText(
            QString("✅ Recorded: %1  [%2 joints, %3 total]")
            .arg(wp.name.c_str())
            .arg(wp.joints.size())
            .arg(waypoints_.size()));

          waypoint_name_edit_->clear();
        }, Qt::QueuedConnection);
      }).detach();
  }

  void TeachPanel::onRecordGripperCommand()
  {
    std::string state =
      gripper_state_selector_->currentText().toStdString();
    std::transform(state.begin(), state.end(), state.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (state != "open" && state != "closed") {
      status_label_->setText("Gripper state must be open or closed.");
      return;
    }

    const int wp_idx = static_cast<int>(waypoints_.size()) + 1;
    std::string wp_name = waypoint_name_edit_->text().toStdString();

    Waypoint wp;
    wp.command_type = "gripper";
    wp.robot = active_robot_;
    wp.gripper_state = state;
    wp.timestamp = getCurrentTimestamp();
    wp.validated = false;
    wp.ee_pose.orientation.w = 1.0;
    wp.name = wp_name.empty()
      ? active_robot_.substr(0, 3) + "_gripper_" + state + "_" + std::to_string(wp_idx)
      : wp_name;

    waypoint_name_edit_->setText(QString::fromStdString(wp.name));
    waypoints_.push_back(wp);
    updateWaypointList();
    status_label_->setText(
      QString("Recorded gripper command: %1 -> %2")
      .arg(wp.robot.c_str())
      .arg(wp.gripper_state.c_str()));
    waypoint_name_edit_->clear();
  }

  void TeachPanel::onGoToWaypoint()
  {
    int row = waypoint_list_->currentRow();
    if (row < 0 || row >= static_cast<int>(waypoints_.size())) {
      return;
    }

    const Waypoint& wp = waypoints_[row];

    if (wp.command_type == "gripper") {
      status_label_->setText(
        QString("Gripper command '%1' is saved for validation/execution.")
        .arg(wp.name.c_str()));
      return;
    }

    if (wp.joints.empty()) {
      status_label_->setText("⚠ Waypoint has no joint data — re-record it.");
      return;
    }

    auto mg = getMoveGroupForRobot(wp.robot);
    if (!mg) {
      initializeMoveItForRobot(wp.robot);
      status_label_->setText(
        QString("MoveIt for %1 is not ready yet. Try again in a moment.")
        .arg(wp.robot.c_str()));
      return;
    }

    status_label_->setText(QString("Planning to %1...").arg(wp.name.c_str()));

    mg->setStartStateToCurrentState();
    mg->setJointValueTarget(wp.joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = mg->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      status_label_->setText(QString("✅ Executing to %1...").arg(wp.name.c_str()));
      mg->execute(plan);
      status_label_->setText(QString("✅ Reached %1").arg(wp.name.c_str()));
    }
    else {
      status_label_->setText(QString("❌ Plan FAILED for %1").arg(wp.name.c_str()));
    }
  }

  void TeachPanel::onDeleteWaypoint()
  {
    int row = waypoint_list_->currentRow();
    if (row < 0 || row >= static_cast<int>(waypoints_.size())) return;
    waypoints_.erase(waypoints_.begin() + row);
    updateWaypointList();
  }

  void TeachPanel::onWaypointSelected(int row)
  {
    (void)row;  // reserved for future preview functionality
  }

  void TeachPanel::onProgramSelected(int index)
  {
    if (!program_selector_ || index < 0 || !program_selector_->isEnabled()) {
      return;
    }

    const auto program = program_selector_->itemText(index);
    if (program.isEmpty()) {
      return;
    }

    active_program_ = program.toStdString();
    program_name_edit_->setText(program);
  }

  void TeachPanel::refreshProgramList()
  {
    if (!program_selector_) {
      return;
    }

    const auto selected_program = program_name_edit_
      ? program_name_edit_->text()
      : QString::fromStdString(active_program_);

    QSignalBlocker blocker(program_selector_);
    program_selector_->clear();

    const auto programs = ProgramManager::listPrograms();
    if (programs.empty()) {
      program_selector_->addItem("No saved programs");
      program_selector_->setEnabled(false);
      return;
    }

    program_selector_->setEnabled(true);
    for (const auto& program : programs) {
      program_selector_->addItem(QString::fromStdString(program));
    }

    int index = program_selector_->findText(selected_program);
    if (index < 0 && !active_program_.empty()) {
      index = program_selector_->findText(QString::fromStdString(active_program_));
    }
    program_selector_->setCurrentIndex(index);
  }

  void TeachPanel::onSaveProgram()
  {
    active_program_ = program_name_edit_->text().toStdString();
    if (active_program_.empty()) {
      status_label_->setText("⚠ Enter a program name first.");
      return;
    }
    bool ok = ProgramManager::saveProgram(active_program_, waypoints_);
    if (ok) {
      refreshProgramList();
    }
    status_label_->setText(ok
      ? QString("💾 Saved: %1.yaml").arg(active_program_.c_str())
      : "❌ Save failed — check disk permissions.");
  }

  void TeachPanel::onLoadProgram()
  {
    active_program_ = program_name_edit_->text().toStdString();
    try {
      waypoints_ = ProgramManager::loadProgram(active_program_);
      updateWaypointList();
      refreshProgramList();
      status_label_->setText(
        QString("📂 Loaded '%1' — %2 waypoints.")
        .arg(active_program_.c_str()).arg(waypoints_.size()));
    }
    catch (const std::exception& e) {
      status_label_->setText(QString("❌ Load failed: %1").arg(e.what()));
    }
  }

  void TeachPanel::onValidateInSim()
  {
    if (waypoints_.empty()) {
      status_label_->setText("No waypoints to validate.");
      return;
    }

    active_program_ = program_name_edit_->text().toStdString();
    if (active_program_.empty()) {
      status_label_->setText("Enter a program name first.");
      return;
    }

    bool ok = ProgramManager::saveProgram(active_program_, waypoints_);
    if (!ok) {
      status_label_->setText("Failed to save program before validation.");
      return;
    }
    refreshProgramList();

    std_msgs::msg::String msg;
    msg.data = active_program_;
    program_validate_pub_->publish(msg);

    status_label_->setText(
      QString("Sent '%1' for twin simulation validation.")
      .arg(active_program_.c_str()));
  }

  void TeachPanel::onExecuteReal()
  {
    active_program_ = program_name_edit_->text().toStdString();
    try {
      waypoints_ = ProgramManager::loadProgram(active_program_);
      updateWaypointList();
    }
    catch (const std::exception& e) {
      status_label_->setText(QString("Load failed before execution: %1").arg(e.what()));
      return;
    }

    bool all_validated = std::all_of(waypoints_.begin(), waypoints_.end(),
      [](const Waypoint& wp) { return wp.validated; });

    if (!all_validated) {
      QMessageBox::warning(this, "Execution Blocked",
        "Not all waypoints are validated.\nRun 'Validate in Sim' first.");
      return;
    }

    auto reply = QMessageBox::question(this, "Execute on Real Hardware",
      QString("Execute '%1' on physical robots?\nMake sure the workspace is clear!")
      .arg(active_program_.c_str()));

    if (reply != QMessageBox::Yes) return;

    std_msgs::msg::String msg;
    msg.data = active_program_;
    program_exec_pub_->publish(msg);

    status_label_->setText(
      QString("⚡ Executing '%1' on real hardware...").arg(active_program_.c_str()));
  }

  // ─────────────────────────────────────────────────────────────
  // Display updates
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::updateCurrentPoseDisplay()
  {
    if (!current_joint_states_) return;

    // Read directly from /joint_states message — no MoveIt2 call needed
    // This avoids getCurrentPose() which triggers the slow state monitor
    const auto& js = *current_joint_states_;

    // Find ABB joints (named joint_1..joint_6) or AR4 (ar4_joint_1..6)
    std::string prefix = (active_robot_ == "abb_irb120") ? "joint_" : "ar4_joint_";

    QString jtext = "J: ";
    int found = 0;
    for (size_t i = 0; i < js.name.size() && found < 6; ++i) {
      if (js.name[i].find(prefix) == 0) {
        jtext += QString("%1° ")
          .arg(js.name[i].back())  // joint number
          + QString("%1 ").arg(js.position[i] * 180.0 / M_PI, 0, 'f', 1);
        found++;
      }
    }
    joint_display_->setText(jtext);
    pose_display_->setText("EE: move robot to update");
  }

  void TeachPanel::updateWaypointList()
  {
    waypoint_list_->clear();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      QString icon = wp.validated ? "✅" : "⬜";
      QString kind = (wp.command_type == "gripper")
        ? QString("gripper:%1").arg(wp.gripper_state.c_str())
        : QString("pose");
      waypoint_list_->addItem(
        QString("%1 #%2  %3  [%4 %5]")
        .arg(icon)
        .arg(i + 1)
        .arg(wp.name.c_str())
        .arg(wp.robot.substr(0, 3).c_str())
        .arg(kind));
    }
  }

  // ─────────────────────────────────────────────────────────────
  // Config persistence (RViz saves/loads these across sessions)
  // ─────────────────────────────────────────────────────────────
  void TeachPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("ActiveRobot", QString::fromStdString(active_robot_));
    config.mapSetValue("ProgramName", QString::fromStdString(active_program_));
  }

  void TeachPanel::load(const rviz_common::Config& config)
  {
    rviz_common::Panel::load(config);
    QString robot, program;
    if (config.mapGetString("ActiveRobot", &robot)) {
      active_robot_ = robot.toStdString();
      int idx = robot_selector_->findText(robot);
      if (idx >= 0) robot_selector_->setCurrentIndex(idx);
    }
    if (config.mapGetString("ProgramName", &program)) {
      active_program_ = program.toStdString();
      program_name_edit_->setText(program);
      refreshProgramList();
    }
  }

  bool TeachPanel::planToWaypoint(const Waypoint& wp)
  {
    if (wp.command_type == "gripper") {
      return wp.gripper_state == "open" || wp.gripper_state == "closed";
    }

    auto mg = getMoveGroupForRobot(wp.robot);
    if (!mg) {
      return false;
    }

    mg->setJointValueTarget(wp.joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }


}  // namespace dual_arms_teach_panel

// ── RViz plugin registration macro ───────────────────────────
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dual_arms_teach_panel::TeachPanel, rviz_common::Panel)
