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

  // ── Inherit sim time from RViz parent node ──────────────────
  // getRosNodeAbstraction() returns a sub-node that does NOT
  // automatically inherit use_sim_time from the RViz node in Jazzy
  if (!node_->has_parameter("use_sim_time")) {
    node_->declare_parameter("use_sim_time", true);
  } else {
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  // Subscribe to joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", 10,
  [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    current_joint_states_ = msg;

    // ── Throttle UI updates to 10Hz max ──────────────────────
    auto now = std::chrono::steady_clock::now();
    auto dt  = std::chrono::duration_cast<std::chrono::milliseconds>(
                 now - last_display_update_).count();
    if (dt < 100) return;  // skip if less than 100ms since last update
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

  // ── MoveIt2 is NOT initialized here anymore ──
  // It will be initialized lazily when first /joint_states arrives
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
}

void TeachPanel::onTeachModeChanged(int index)
{
  jog_mode_ = index;
  if (jog_mode_ == 1) {
    jog_timer_->start();
  } else {
    jog_timer_->stop();
    jog_cmd_ = geometry_msgs::msg::Twist();  // zero velocities
  }
}

void TeachPanel::onJogTick()
{
  // Called at 50 Hz while servo jog tab is active.
  // The actual publishing is done by ServoJogWidget via keyPress/keyRelease.
  // This tick is reserved for future gamepad polling.
}

void TeachPanel::onRecordWaypoint()
{
  if (!abb_mg_ || !ar4_mg_) {
  status_label_->setText("⚠ MoveIt2 not ready yet — wait for 'Ready' status.");
  return;
  }

  if (!current_joint_states_) {
    status_label_->setText("⚠ No joint states received yet.");
    return;
  }

  Waypoint wp;
  wp.name      = waypoint_name_edit_->text().toStdString();
  wp.robot     = active_robot_;
  wp.timestamp = getCurrentTimestamp();
  wp.validated = false;

  auto& mg = (active_robot_ == "abb_irb120") ? abb_mg_ : ar4_mg_;
  wp.joints  = mg->getCurrentJointValues();
  wp.ee_pose = mg->getCurrentPose().pose;

  if (wp.name.empty()) {
    wp.name = active_robot_.substr(0, 3) + "_wp_" + std::to_string(waypoints_.size() + 1);
    waypoint_name_edit_->setText(QString::fromStdString(wp.name));
  }

  waypoints_.push_back(wp);
  updateWaypointList();

  status_label_->setText(
    QString("✅ Recorded: %1  [%2 total]")
      .arg(wp.name.c_str()).arg(waypoints_.size()));

  waypoint_name_edit_->clear();
}

void TeachPanel::onGoToWaypoint()
{
  if (!abb_mg_ || !ar4_mg_) {
  status_label_->setText("⚠ MoveIt2 not ready yet — wait for 'Ready' status.");
  return;
  }

  int row = waypoint_list_->currentRow();
  if (row < 0 || row >= static_cast<int>(waypoints_.size())) return;

  const Waypoint& wp = waypoints_[row];
  auto& mg = (wp.robot == "abb_irb120") ? abb_mg_ : ar4_mg_;

  status_label_->setText(QString("Planning to %1...").arg(wp.name.c_str()));

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

void TeachPanel::onSetFromXYZ()
{
  // Build a target pose from the spinbox values and send to MoveIt2
  geometry_msgs::msg::Pose target;
  target.position.x = x_spin_->value();
  target.position.y = y_spin_->value();
  target.position.z = z_spin_->value();

  // Convert RPY (degrees) to quaternion
  double rx = rx_spin_->value() * M_PI / 180.0;
  double ry = ry_spin_->value() * M_PI / 180.0;
  double rz = rz_spin_->value() * M_PI / 180.0;

  double cy = cos(rz * 0.5), sy = sin(rz * 0.5);
  double cp = cos(ry * 0.5), sp = sin(ry * 0.5);
  double cr = cos(rx * 0.5), sr = sin(rx * 0.5);

  target.orientation.w = cr * cp * cy + sr * sp * sy;
  target.orientation.x = sr * cp * cy - cr * sp * sy;
  target.orientation.y = cr * sp * cy + sr * cp * sy;
  target.orientation.z = cr * cp * sy - sr * sp * cy;

  auto& mg = (active_robot_ == "abb_irb120") ? abb_mg_ : ar4_mg_;
  mg->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = mg->plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    status_label_->setText("✅ XYZ pose reachable — press Record to save.");
    mg->execute(plan);
  } else {
    status_label_->setText("❌ XYZ pose not reachable. Adjust values.");
  }
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
  status_label_->setText("⚠ MoveIt2 not ready yet — wait for 'Ready' status.");
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
