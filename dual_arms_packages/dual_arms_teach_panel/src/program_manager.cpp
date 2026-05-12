#include "dual_arms_teach_panel/program_manager.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace fs = std::filesystem;

namespace dual_arms_teach_panel {   // ← was MISSING — caused all "not declared" errors

  // ─────────────────────────────────────────────────────────────
  std::string ProgramManager::getStorageDir() {
    auto home = std::string(getenv("HOME"));
    auto dir = home + "/.ros/dt_programs";
    fs::create_directories(dir);
    return dir;
  }

  // ─────────────────────────────────────────────────────────────
  std::string ProgramManager::getCurrentTimestamp() {
    // Returns e.g. "2026-03-11T14:32:01"
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%dT%H:%M:%S");
    return ss.str();
  }

  // ─────────────────────────────────────────────────────────────
  bool ProgramManager::saveProgram(
    const std::string& name,
    const std::vector<Waypoint>& waypoints)
  {

    bool program_validated = !waypoints.empty();
    for (const auto& wp : waypoints) {
      if (!wp.validated) {
        program_validated = false;
        break;
      }
    }

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "program_name" << YAML::Value << name;
    out << YAML::Key << "created" << YAML::Value << getCurrentTimestamp();
    out << YAML::Key << "validated" << YAML::Value << program_validated;
    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto& wp = waypoints[i];
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << static_cast<int>(i) + 1;
      out << YAML::Key << "type" << YAML::Value
          << (wp.command_type.empty() ? "waypoint" : wp.command_type);
      out << YAML::Key << "name" << YAML::Value << wp.name;
      out << YAML::Key << "robot" << YAML::Value << wp.robot;
      out << YAML::Key << "validated" << YAML::Value << wp.validated;
      out << YAML::Key << "timestamp" << YAML::Value << wp.timestamp;
      if (wp.command_type == "gripper") {
        out << YAML::Key << "gripper_state" << YAML::Value << wp.gripper_state;
      }

      // Joint positions
      out << YAML::Key << "joints" << YAML::Value << YAML::BeginSeq;
      for (double j : wp.joints) out << j;
      out << YAML::EndSeq;

      // End-effector pose
      out << YAML::Key << "ee_pose" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << wp.ee_pose.position.x;
      out << YAML::Key << "y" << YAML::Value << wp.ee_pose.position.y;
      out << YAML::Key << "z" << YAML::Value << wp.ee_pose.position.z;
      out << YAML::Key << "qx" << YAML::Value << wp.ee_pose.orientation.x;
      out << YAML::Key << "qy" << YAML::Value << wp.ee_pose.orientation.y;
      out << YAML::Key << "qz" << YAML::Value << wp.ee_pose.orientation.z;
      out << YAML::Key << "qw" << YAML::Value << wp.ee_pose.orientation.w;
      out << YAML::EndMap;

      out << YAML::EndMap;
    }

    out << YAML::EndSeq << YAML::EndMap;

    auto path = getStorageDir() + "/" + name + ".yaml";
    std::ofstream file(path);
    file << out.c_str();
    return file.good();
  }

  // ─────────────────────────────────────────────────────────────
  std::vector<Waypoint> ProgramManager::loadProgram(
    const std::string& name)
  {
    auto path = getStorageDir() + "/" + name + ".yaml";

    if (!fs::exists(path)) {
      throw std::runtime_error("Program not found: " + path);
    }

    YAML::Node root = YAML::LoadFile(path);
    std::vector<Waypoint> waypoints;

    for (const auto& node : root["waypoints"]) {
      Waypoint wp;
      wp.command_type = node["type"] ? node["type"].as<std::string>() : "waypoint";
      wp.name = node["name"].as<std::string>();
      wp.robot = node["robot"].as<std::string>();
      wp.validated = node["validated"].as<bool>();
      wp.timestamp = node["timestamp"].as<std::string>();
      if (node["gripper_state"]) {
        wp.gripper_state = node["gripper_state"].as<std::string>();
      }

      if (node["joints"]) {
        for (auto j : node["joints"]) {
          wp.joints.push_back(j.as<double>());
        }
      }

      if (node["ee_pose"]) {
        const auto& ep = node["ee_pose"];
        wp.ee_pose.position.x = ep["x"].as<double>();
        wp.ee_pose.position.y = ep["y"].as<double>();
        wp.ee_pose.position.z = ep["z"].as<double>();
        wp.ee_pose.orientation.x = ep["qx"].as<double>();
        wp.ee_pose.orientation.y = ep["qy"].as<double>();
        wp.ee_pose.orientation.z = ep["qz"].as<double>();
        wp.ee_pose.orientation.w = ep["qw"].as<double>();
      }

      waypoints.push_back(wp);
    }

    return waypoints;
  }

  // ─────────────────────────────────────────────────────────────
  std::vector<std::string> ProgramManager::listPrograms() {
    std::vector<std::string> names;
    for (const auto& entry : fs::directory_iterator(getStorageDir())) {
      if (entry.path().extension() == ".yaml") {
        names.push_back(entry.path().stem().string());
      }
    }
    std::sort(names.begin(), names.end());
    return names;
  }

}  // namespace dual_arms_teach_panel
