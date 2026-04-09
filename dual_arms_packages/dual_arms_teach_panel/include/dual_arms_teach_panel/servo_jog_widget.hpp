#pragma once

#include "dual_arms_teach_panel/teach_panel.hpp"   // for Waypoint

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QWidget>
#include <QKeyEvent>
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>

namespace dual_arms_teach_panel {

// ─────────────────────────────────────────────────────────────
// ServoJogWidget
//
// A QWidget that:
//   - Captures keyboard events (WASD / QE / ZX) while focused
//   - Publishes geometry_msgs/Twist to /servo_node/delta_twist_cmds
//     which moveit_servo converts to joint velocity commands
//   - Emits recordRequested() when Space is pressed
//
// Mount this widget inside the "Servo Jog" tab of TeachPanel.
// ─────────────────────────────────────────────────────────────
class ServoJogWidget : public QWidget {
  Q_OBJECT

public:
  explicit ServoJogWidget(rclcpp::Node::SharedPtr node,
                          QWidget* parent = nullptr);
  ~ServoJogWidget() override = default;

signals:
  // Emitted when Space bar is pressed — TeachPanel records the waypoint
  void recordRequested();
  // Emitted on every key press/release with the current jog twist.
  // TeachPanel uses this to move the interactive marker (not the real robot).
  void jogTwist(const geometry_msgs::msg::Twist& twist);

protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

private:
  void buildUI();
  void publishJogCommand();

  // ROS2 node kept for future use (e.g. parameter reads)
  rclcpp::Node::SharedPtr node_;

  // Current jog command (zeroed on key release)
  geometry_msgs::msg::Twist jog_cmd_;

  // UI
  QLabel* hint_label_;
};

}  // namespace dual_arms_teach_panel