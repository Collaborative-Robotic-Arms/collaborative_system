#pragma once

// Must include teach_panel.hpp FIRST so Waypoint struct is visible
#include "dual_arms_teach_panel/teach_panel.hpp"

#include <string>
#include <vector>

namespace dual_arms_teach_panel {

  // ─────────────────────────────────────────────────────────────
  // ProgramManager
  //
  // Static utility class for saving/loading waypoint programs
  // as YAML files in ~/.ros/dt_programs/
  //
  // All methods are static — no instance needed.
  // Usage:
  //   ProgramManager::saveProgram("wall_v1", waypoints_);
  //   auto wps = ProgramManager::loadProgram("wall_v1");
  //   auto names = ProgramManager::listPrograms();
  // ─────────────────────────────────────────────────────────────
  class ProgramManager {
  public:
    // Save a named program (waypoint list) to YAML.
    // Returns true on success.
    static bool saveProgram(
      const std::string& name,
      const std::vector<Waypoint>& waypoints);

    // Load a named program from YAML.
    // Throws std::runtime_error if file not found.
    static std::vector<Waypoint> loadProgram(
      const std::string& name);

    // List all program names available in the storage dir
    // (returns stems only, e.g. "wall_v1", not full paths).
    static std::vector<std::string> listPrograms();

    // Returns the full path to the storage directory,
    // creating it if it does not exist.
    static std::string getStorageDir();

    // ISO-8601 timestamp string for the current time,
    // e.g. "2026-03-11T14:32:01"
    static std::string getCurrentTimestamp();
  };

}  // namespace dual_arms_teach_panel
