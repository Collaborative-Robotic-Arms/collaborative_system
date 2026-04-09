#include "dual_arms_teach_panel/teach_panel.hpp"
#include "dual_arms_teach_panel/program_manager.hpp"
#include "dual_arms_teach_panel/servo_jog_widget.hpp"
#include "dual_arms_teach_panel/waypoint_list_widget.hpp"

#include <QMessageBox>
#include <QGroupBox>
#include <QScrollArea>
#include <QSizePolicy>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <thread>
#include <cmath>

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
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  if (!node_->has_parameter("use_sim_time")) {
    node_->declare_parameter("use_sim_time", true);
  } else {
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  // ── Joint states ─────────────────────────────────────────────
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      current_joint_states_ = msg;

      auto now = std::chrono::steady_clock::now();
      auto dt  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - last_display_update_).count();
      if (dt < 100) return;
      last_display_update_ = now;

      if (!moveit_initialized_) {
        initializeMoveIt();
        return;
      }
      QMetaObject::invokeMethod(
        this, &TeachPanel::updateCurrentPoseDisplay, Qt::QueuedConnection);
    });

  program_exec_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/dt/program_execute", 1);
  estop_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/dt/estop", 1);

  // Inside TeachPanel::onInitialize()
  move_marker_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
  "/rviz/moveit/move_marker/goal", 10);

  // ── Interactive marker feedback subscriber ────────────────────
  // The MotionPlanning display publishes here whenever the user drags
  // the 3D marker. We store the EE pose so Record can use it without
  // any blocking MoveIt2 call.
  marker_feedback_sub_ =
    node_->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
      "/rviz_moveit_motion_planning_display"
      "/robot_interaction_interactive_marker_topic/feedback",
      10,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
        if (msg->event_type !=
            visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) return;
        latest_marker_pose_  = msg->pose;
        marker_pose_received_ = true;
        // Keep servo_target_pose_ in sync with the real marker position.
        // This way, when the user switches to Servo Jog, jogging starts
        // from wherever the marker currently is — not from a stale position.
        if (!servo_initialized_) {
          servo_target_pose_ = msg->pose;
        }
      });

  // ── ServoJogWidget ────────────────────────────────────────────
  // Instantiated here (not in buildUI) because it needs node_.
  if (servo_tab_widget_) {
    servo_jog_widget_ = new ServoJogWidget(node_, servo_tab_widget_);
    auto* tab_layout = qobject_cast<QVBoxLayout*>(servo_tab_widget_->layout());
    if (tab_layout) {
      QLayoutItem* item = tab_layout->takeAt(0);   // remove placeholder label
      if (item) { delete item->widget(); delete item; }
      tab_layout->insertWidget(0, servo_jog_widget_);
    }
    connect(servo_jog_widget_, &ServoJogWidget::recordRequested,
            this, &TeachPanel::onRecordWaypoint);
    connect(servo_jog_widget_, &ServoJogWidget::jogTwist,
            this, &TeachPanel::onServoJogTwist);
  }

  status_label_->setText("Waiting for joint states...");
}

void TeachPanel::initializeMoveIt()
{
  if (moveit_initialized_) return;
  moveit_initialized_ = true;

  std::thread([this]() {
    try {
      // ── Tell this node to use sim time ──────────────────────
      node_->set_parameter(rclcpp::Parameter("use_sim_time", true));

      moveit::planning_interface::MoveGroupInterface::Options abb_opts("irb120_arm");
      moveit::planning_interface::MoveGroupInterface::Options ar4_opts("ar_manipulator");

      abb_mg_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, abb_opts);
      ar4_mg_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, ar4_opts);

      QMetaObject::invokeMethod(this, [this]() {
        status_label_->setText("Ready. Choose a teach mode and position the robot.");
      }, Qt::QueuedConnection);

    } catch (const std::exception& e) {
      QMetaObject::invokeMethod(this, [this, e]() {
        status_label_->setText(QString("⚠ MoveIt2 init failed: %1").arg(e.what()));
      }, Qt::QueuedConnection);
    }
  }).detach();
}


// ─────────────────────────────────────────────────────────────
// Timestamp helper
// ─────────────────────────────────────────────────────────────
std::string TeachPanel::getCurrentTimestamp()
{
  auto now = std::chrono::system_clock::now();
  auto t   = std::chrono::system_clock::to_time_t(now);
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
  robot_selector_->addItems({"abb_irb120", "ar4_mk3"});
  connect(robot_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &TeachPanel::onRobotChanged);
  robot_row->addWidget(robot_selector_);
  root_layout->addLayout(robot_row);

  // ── Current pose display ──────────────────────────────────
  auto* pose_group = new QGroupBox("Current Pose");
  auto* pose_layout = new QVBoxLayout(pose_group);
  pose_display_  = new QLabel("EE:  waiting...");
  joint_display_ = new QLabel("J:   waiting...");
  pose_display_->setFont(QFont("Courier", 9));
  joint_display_->setFont(QFont("Courier", 9));
  pose_layout->addWidget(pose_display_);
  pose_layout->addWidget(joint_display_);
  root_layout->addWidget(pose_group);

  // ── Teach mode tabs ───────────────────────────────────────
  teach_tabs_ = new QTabWidget();
  auto* marker_tab = new QWidget();
  auto* servo_tab  = new QWidget();
  auto* manual_tab = new QWidget();
  buildMarkerDragTab(marker_tab);
  buildServoJogTab(servo_tab);
  buildManualXYZTab(manual_tab);
  teach_tabs_->addTab(marker_tab, "Marker Drag");
  teach_tabs_->addTab(servo_tab,  "Servo Jog");
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
  auto* goto_btn   = new QPushButton("▶ Go To");
  auto* delete_btn = new QPushButton("✗ Delete");
  connect(goto_btn,   &QPushButton::clicked, this, &TeachPanel::onGoToWaypoint);
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

  auto* prog_btn_row = new QHBoxLayout();
  auto* save_btn     = new QPushButton("💾 Save YAML");
  auto* load_btn     = new QPushButton("📂 Load");
  auto* validate_btn = new QPushButton("▶ Validate in Sim");
  auto* execute_btn  = new QPushButton("⚡ Execute Real");

  save_btn->setStyleSheet("background:#1D4ED8; color:white; border-radius:3px; padding:5px;");
  validate_btn->setStyleSheet("background:#6D28D9; color:white; border-radius:3px; padding:5px;");
  execute_btn->setStyleSheet("background:#991B1B; color:white; border-radius:3px; padding:5px;");

  connect(save_btn,     &QPushButton::clicked, this, &TeachPanel::onSaveProgram);
  connect(load_btn,     &QPushButton::clicked, this, &TeachPanel::onLoadProgram);
  connect(validate_btn, &QPushButton::clicked, this, &TeachPanel::onValidateInSim);
  connect(execute_btn,  &QPushButton::clicked, this, &TeachPanel::onExecuteReal);

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
  layout->addWidget(new QLabel("X (m):"),  0, 0);  x_spin_  = new QDoubleSpinBox(); x_spin_->setRange(-2.0, 2.0); x_spin_->setSingleStep(0.01); layout->addWidget(x_spin_,  0, 1);
  layout->addWidget(new QLabel("Y (m):"),  1, 0);  y_spin_  = new QDoubleSpinBox(); y_spin_->setRange(-2.0, 2.0); y_spin_->setSingleStep(0.01); layout->addWidget(y_spin_,  1, 1);
  layout->addWidget(new QLabel("Z (m):"),  2, 0);  z_spin_  = new QDoubleSpinBox(); z_spin_->setRange(-2.0, 2.0); z_spin_->setSingleStep(0.01); layout->addWidget(z_spin_,  2, 1);
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
  // Reset stored marker pose — new robot, new marker
  marker_pose_received_ = false;
  servo_target_pose_ = geometry_msgs::msg::Pose();
  servo_target_pose_.orientation.w = 1.0;
}

void TeachPanel::onTeachModeChanged(int index)
{
  jog_mode_ = index;
  if (jog_mode_ == 1) {
    // Entering Servo Jog tab — reset so servo_target_pose_ re-seeds
    // from the current marker position (picked up via feedback subscriber).
    servo_initialized_ = false;
    jog_timer_->start();
  } else {
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
    } else if (current_joint_states_) {
      // Fallback: use actual robot position as starting point
      servo_target_pose_ = geometry_msgs::msg::Pose();
      servo_target_pose_.position.x    = 0.3;
      servo_target_pose_.position.z    = 0.3;
      servo_target_pose_.orientation.w = 1.0;
    } else {
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
    ps.header.stamp    = node_->now();
    ps.header.frame_id = (active_robot_ == "abb_irb120") ? "base_link" : "ar4_base_link";
    ps.pose            = servo_target_pose_;
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
    ps.header.stamp    = node_->now();
    ps.header.frame_id = (active_robot_ == "abb_irb120") ? "base_link" : "ar4_base_link";
    ps.pose            = manual_target_pose_;
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
  if (!moveit_initialized_ || !abb_mg_ || !ar4_mg_) {
    status_label_->setText("⚠ MoveIt2 not ready yet — wait for 'Ready' status.");
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
    have_pose   = true;
  } else if (jog_mode_ == 1) {
    target_pose = servo_target_pose_;
    have_pose   = true;
  } else if (jog_mode_ == 2) {
    target_pose = manual_target_pose_;
    have_pose   = true;
  }

  if (!have_pose) {
    status_label_->setText("⚠ No target pose set.");
    return;
  }

  // Snapshot waypoint fields now (on Qt thread, safe)
  std::string wp_name   = waypoint_name_edit_->text().toStdString();
  std::string wp_robot  = active_robot_;
  std::string wp_ts     = getCurrentTimestamp();
  int         wp_idx    = static_cast<int>(waypoints_.size()) + 1;

  status_label_->setText("⏳ Solving IK...");

  // ── Run IK on a background thread ────────────────────────────
  // setApproximateJointValueTarget internally calls IK. We must NOT
  // call it on the Qt main thread — it can block and cause the segfault.
  std::thread([this, target_pose, wp_name, wp_robot, wp_ts, wp_idx]() {
    auto& mg = (wp_robot == "abb_irb120") ? abb_mg_ : ar4_mg_;

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
      wp.joints    = joints;
      wp.ee_pose   = target_pose;
      wp.robot     = wp_robot;
      wp.timestamp = wp_ts;
      wp.validated = false;
      wp.name      = wp_name.empty()
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

void TeachPanel::onGoToWaypoint()
{
  if (!abb_mg_ || !ar4_mg_) {
    status_label_->setText("⚠ MoveIt2 not ready yet.");
    return;
  }

  int row = waypoint_list_->currentRow();
  if (row < 0 || row >= static_cast<int>(waypoints_.size())) return;

  const Waypoint& wp = waypoints_[row];

  if (wp.joints.empty()) {
    status_label_->setText("⚠ Waypoint has no joint data — re-record it.");
    return;
  }

  auto& mg = (wp.robot == "abb_irb120") ? abb_mg_ : ar4_mg_;
  status_label_->setText(QString("Planning to %1...").arg(wp.name.c_str()));

  mg->setStartStateToCurrentState();
  mg->setJointValueTarget(wp.joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = mg->plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    status_label_->setText(QString("✅ Executing to %1...").arg(wp.name.c_str()));
    mg->execute(plan);
    status_label_->setText(QString("✅ Reached %1").arg(wp.name.c_str()));
  } else {
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


void TeachPanel::onSaveProgram()
{
  active_program_ = program_name_edit_->text().toStdString();
  if (active_program_.empty()) {
    status_label_->setText("⚠ Enter a program name first.");
    return;
  }
  bool ok = ProgramManager::saveProgram(active_program_, waypoints_);
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
    status_label_->setText(
      QString("📂 Loaded '%1' — %2 waypoints.")
        .arg(active_program_.c_str()).arg(waypoints_.size()));
  } catch (const std::exception& e) {
    status_label_->setText(QString("❌ Load failed: %1").arg(e.what()));
  }
}

void TeachPanel::onValidateInSim()
{
  if (!abb_mg_ || !ar4_mg_) {
    status_label_->setText("⚠ MoveIt2 not ready yet.");
    return;
  }
  if (waypoints_.empty()) {
    status_label_->setText("⚠ No waypoints to validate.");
    return;
  }

  status_label_->setText("🔄 Validating waypoints in simulation...");
  bool all_ok = true;

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    auto& wp = waypoints_[i];
    auto& mg = (wp.robot == "abb_irb120") ? abb_mg_ : ar4_mg_;

    if (wp.joints.empty()) {
      wp.validated = false;
      status_label_->setText(
        QString("❌ Waypoint #%1 '%2' has no joint data — re-record it.")
          .arg(i + 1).arg(wp.name.c_str()));
      all_ok = false;
      break;
    }

    // Use planning scene state instead of hardware state monitor
    // This avoids the sim clock timeout in getCurrentRobotState()
    mg->setStartStateToCurrentState();
    mg->setJointValueTarget(wp.joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = mg->plan(plan);
    wp.validated = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!wp.validated) {
      all_ok = false;
      status_label_->setText(
        QString("❌ Validation FAILED at waypoint #%1: %2")
          .arg(i + 1).arg(wp.name.c_str()));
      break;
    }
  }

  updateWaypointList();

  if (all_ok) {
    status_label_->setText(
      QString("✅ All %1 waypoints validated.").arg(waypoints_.size()));
    ProgramManager::saveProgram(active_program_, waypoints_);
  }
}

void TeachPanel::onExecuteReal()
{
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
    waypoint_list_->addItem(
      QString("%1 #%2  %3  [%4]")
        .arg(icon)
        .arg(i + 1)
        .arg(wp.name.c_str())
        .arg(wp.robot.substr(0, 3).c_str()));
  }
}

// ─────────────────────────────────────────────────────────────
// Config persistence (RViz saves/loads these across sessions)
// ─────────────────────────────────────────────────────────────
void TeachPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ActiveRobot",  QString::fromStdString(active_robot_));
  config.mapSetValue("ProgramName",  QString::fromStdString(active_program_));
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
  }
}

bool TeachPanel::planToWaypoint(const Waypoint& wp)
{
  auto& mg = (wp.robot == "abb_irb120") ? abb_mg_ : ar4_mg_;
  mg->setJointValueTarget(wp.joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  return mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

void TeachPanel::onJointStatesReceived(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_states_ = msg;
  updateCurrentPoseDisplay();
}

}  // namespace dual_arms_teach_panel

// ── RViz plugin registration macro ───────────────────────────
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dual_arms_teach_panel::TeachPanel, rviz_common::Panel)