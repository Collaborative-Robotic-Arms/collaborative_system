#pragma once

#include "dual_arms_teach_panel/teach_panel.hpp"  // for Waypoint

#include <QWidget>
#include <QListWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <vector>
#include <string>

namespace dual_arms_teach_panel {

// ─────────────────────────────────────────────────────────────
// WaypointListWidget
//
// A thin wrapper around QListWidget that displays the
// waypoint list with ✅/❌ validation status icons and
// robot labels.  TeachPanel owns this and calls refresh()
// whenever the waypoints_ vector changes.
// ─────────────────────────────────────────────────────────────
class WaypointListWidget : public QWidget {
  Q_OBJECT

public:
  explicit WaypointListWidget(QWidget* parent = nullptr);
  ~WaypointListWidget() override = default;

  // Rebuild the list from the given waypoints vector
  void refresh(const std::vector<Waypoint>& waypoints);

  // Returns the index of the currently selected row, or -1
  int selectedRow() const;

signals:
  void rowSelected(int row);
  void goToRequested(int row);
  void deleteRequested(int row);

private slots:
  void onItemDoubleClicked(QListWidgetItem* item);

private:
  QListWidget* list_;
};

}  // namespace dual_arms_teach_panel
