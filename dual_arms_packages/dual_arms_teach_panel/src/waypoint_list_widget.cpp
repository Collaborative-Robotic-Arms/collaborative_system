#include "dual_arms_teach_panel/waypoint_list_widget.hpp"

#include <QListWidgetItem>

namespace dual_arms_teach_panel {

WaypointListWidget::WaypointListWidget(QWidget* parent)
  : QWidget(parent)
{
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  list_ = new QListWidget();
  connect(list_, &QListWidget::itemDoubleClicked,
          this, &WaypointListWidget::onItemDoubleClicked);
  connect(list_, &QListWidget::currentRowChanged,
          this, &WaypointListWidget::rowSelected);

  layout->addWidget(list_);
  setLayout(layout);
}

void WaypointListWidget::refresh(const std::vector<Waypoint>& waypoints)
{
  list_->clear();
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& wp = waypoints[i];
    QString icon = wp.validated ? "✅" : "⬜";
    auto* item = new QListWidgetItem(
      QString("%1 #%2  %3  [%4]")
        .arg(icon)
        .arg(i + 1)
        .arg(wp.name.c_str())
        .arg(wp.robot.substr(0, 3).c_str()));
    list_->addItem(item);
  }
}

int WaypointListWidget::selectedRow() const
{
  return list_->currentRow();
}

void WaypointListWidget::onItemDoubleClicked(QListWidgetItem* item)
{
  int row = list_->row(item);
  emit goToRequested(row);
}

}  // namespace dual_arms_teach_panel
