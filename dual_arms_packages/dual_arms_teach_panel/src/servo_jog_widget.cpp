#include "dual_arms_teach_panel/servo_jog_widget.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QFont>

namespace dual_arms_teach_panel {

ServoJogWidget::ServoJogWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
  : QWidget(parent), node_(node)
{
  // Publish to moveit_servo's twist command topic
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/servo_node/delta_twist_cmds", 10);

  buildUI();

  // Required so keyPressEvent is delivered to this widget
  setFocusPolicy(Qt::StrongFocus);
}

void ServoJogWidget::buildUI()
{
  auto* layout = new QVBoxLayout(this);

  hint_label_ = new QLabel(
    "<b>Keyboard Controls</b><br>"
    "<code>W/S</code> — X +/-<br>"
    "<code>A/D</code> — Y +/-<br>"
    "<code>R/F</code> — Z +/-<br>"
    "<code>Q/E</code> — Rz +/-<br>"
    "<code>Z/X</code> — Rx +/-<br>"
    "<code>SPACE</code> — Record waypoint<br><br>"
    "<i>Click here first to focus</i>");
  hint_label_->setStyleSheet("background:#F8FAFC; padding:8px; border-radius:4px;");
  hint_label_->setTextFormat(Qt::RichText);
  layout->addWidget(hint_label_);
  layout->addStretch();

  setLayout(layout);
}

// ─────────────────────────────────────────────────────────────
void ServoJogWidget::keyPressEvent(QKeyEvent* event)
{
  if (event->isAutoRepeat()) return;  // ignore key-held repeats

  const double LIN_VEL = 0.05;  // m/s
  const double ROT_VEL = 0.30;  // rad/s

  switch (event->key()) {
    case Qt::Key_W: jog_cmd_.linear.x  =  LIN_VEL; break;
    case Qt::Key_S: jog_cmd_.linear.x  = -LIN_VEL; break;
    case Qt::Key_A: jog_cmd_.linear.y  =  LIN_VEL; break;
    case Qt::Key_D: jog_cmd_.linear.y  = -LIN_VEL; break;
    case Qt::Key_R: jog_cmd_.linear.z  =  LIN_VEL; break;
    case Qt::Key_F: jog_cmd_.linear.z  = -LIN_VEL; break;
    case Qt::Key_Q: jog_cmd_.angular.z =  ROT_VEL; break;
    case Qt::Key_E: jog_cmd_.angular.z = -ROT_VEL; break;
    case Qt::Key_Z: jog_cmd_.angular.x =  ROT_VEL; break;
    case Qt::Key_X: jog_cmd_.angular.x = -ROT_VEL; break;
    case Qt::Key_Space:
      emit recordRequested();
      return;  // don't publish a twist for Space
    default:
      QWidget::keyPressEvent(event);
      return;
  }
  publishJogCommand();
}

void ServoJogWidget::keyReleaseEvent(QKeyEvent* event)
{
  if (event->isAutoRepeat()) return;

  // Zero the axis that was released
  switch (event->key()) {
    case Qt::Key_W: case Qt::Key_S: jog_cmd_.linear.x  = 0.0; break;
    case Qt::Key_A: case Qt::Key_D: jog_cmd_.linear.y  = 0.0; break;
    case Qt::Key_R: case Qt::Key_F: jog_cmd_.linear.z  = 0.0; break;
    case Qt::Key_Q: case Qt::Key_E: jog_cmd_.angular.z = 0.0; break;
    case Qt::Key_Z: case Qt::Key_X: jog_cmd_.angular.x = 0.0; break;
    default:
      QWidget::keyReleaseEvent(event);
      return;
  }
  publishJogCommand();
}

void ServoJogWidget::publishJogCommand()
{
  twist_pub_->publish(jog_cmd_);
}

}  // namespace dual_arms_teach_panel
