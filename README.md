# Collaborative_system

This repository uses **Git Submodules** to manage multiple internal components and dependencies. To ensure all submodules are downloaded correctly, please follow the installation steps below.

## 🛠 Installation & Setup

These instructions assume you are setting up a standard ROS 2 style workspace.

### 1. Clone the Repository

Use the `--recursive` flag to automatically initialize and update every submodule in the repository.

```bash
# Create and enter the source directory
mkdir -p collab_ws/src
cd collab_ws/src

# Clone the repository and all submodules
git clone --recurse-submodules https://github.com/Collaborative-Robotic-Arms/collaborative_system.git .

```

### 2. Build the Workspace

Once cloned, navigate back to the workspace root to build the project.

```bash
cd ..
colcon build

```

---

## 💡 Quick Tips for Submodules

If you've already cloned the repo without the `--recursive` flag, or if someone else updates a submodule, use these commands:

* **To initialize submodules after cloning:**
```bash
git submodule update --init --recursive

```


* **To pull the latest changes for all submodules:**
```bash
git submodule update --recursive --remote

```
# 🦾 Dual-Arm Collaborative Robot System

> **ABB IRB120 + AR4 Mk3 | ROS2 Jazzy | MoveIt2 | Gazebo | Firebase**

A full-stack robotic assembly cell where two collaborative arms work together to pick and place bricks according to a user-designed layout. The operator draws the assembly plan in a web browser — the robots build it.

---

## 📋 Table of Contents

- [System Overview](#system-overview)
- [Prerequisites](#prerequisites)
- [Component 1 — GUI](#1-gui)
- [Component 2 — Robot Environment](#2-robot-environment)
- [Component 3 — Detection & Grasping](#3-detection--grasping)
- [Component 4 — Supervisor](#4-supervisor)
- [Full System Checklists](#full-system-checklists)
- [Diagnostics & Verification](#diagnostics--verification)
- [Launch Arguments Reference](#launch-arguments-reference)
- [Future — Single Script Launch](#future--single-script-launch)

---

## System Overview

The system has **4 independent components**, each running in its own terminal. Launch them in this order:

| Order | Component | Package | Purpose |
|-------|-----------|---------|---------|
| 1st | **GUI** | `GUI_PACKAGE` + `ros2_gui_bridge` | Operator draws brick layout → Firebase → ROS2 |
| 2nd | **Robot Environment** | `dual_arms` → `master_launch.py` | Robots, MoveIt2, RViz, controllers |
| 3rd | **Detection & Grasping** | `detection_grasping_bringup` | Camera, YOLO detection, grasp planning |
| 4th | **Supervisor** | `supervisor_package` → `supervisor2` | Orchestrates the full assembly sequence |

> ⚠️ **Open 4 terminals before starting.** Never close a terminal while the system is running — use `Ctrl+C` to shut down gracefully.

---

## Prerequisites

### Software
- Ubuntu 24.04
- ROS2 Jazzy
- Node.js + npm (for GUI)
- Python 3 with `firebase-admin`

### Build the workspace
```bash
cd ~/collab_ws
colcon build --allow-overriding abb_robot_msgs ros2srrc_robots
source install/setup.bash
```

### Hardware (real mode only)
| Hardware | Connection | Address |
|----------|-----------|---------|
| ABB IRC5 Controller | Ethernet (enp7s0) | `192.168.125.1` |
| AR4 Teensy | USB Serial | `/dev/ttyACM0` |
| AR4 Gripper (Arduino) | USB Serial | `/dev/ttyUSB0` |
| Intel RealSense D4xx | USB 3.0 | Auto-detected |

```bash
# Verify ABB network
ping 192.168.125.1

# Verify AR4 Teensy
lsusb | grep "16c0:0483"
ls /dev/ttyACM*

# Grant serial port access (run once, then logout/login)
sudo usermod -aG dialout $USER
```

---

## 1. GUI

The GUI has two parts: the **web application** (React Native/Expo) where the operator designs the brick layout, and the **ROS2 bridge** that receives the design and feeds it into the robot pipeline.

### Part A — Web Application

**Terminal 1**

```bash
# Step 1 — Navigate to GUI package
cd ~/collab_ws/GUI_PACKAGE

# Step 2 — Install dependencies (first time only)
npm install

# Step 3 — Start the Expo development server
npm start
```

The Expo CLI menu will appear:
```
› Metro waiting on exp://192.168.1.180:8081
› Web is waiting on http://localhost:8081
› Press s │ switch to development build
› Press w │ open web
```

```bash
# Step 4 — Switch to web mode (press these keys in the Expo terminal)
s    # switch to Go mode
w    # open web browser
```

> The browser opens automatically at `http://localhost:8081`

```
# Step 5 — Login credentials
Email:    collabarms@gmail.com
Password: collabarms@2025
```

> **After login:** Design the brick layout using the drag-and-drop grid. Select shapes (I, L, T, Z), rotate and place them on the grid layers. Press **Submit** when done — this uploads the plan to Firebase Firestore and triggers the robot pipeline.

---

### Part B — ROS2 GUI Bridge

**Terminal 2**

```bash
source ~/collab_ws/install/setup.bash
ros2 launch ros2_gui_bridge gui_system.launch.py
```

**Expected output:**
```
[firestore_bridge.py]   --- FIRESTORE BRIDGE STARTING ---
[firestore_bridge.py]   Connected! Listening to 'shapes'...
[brick_processor2.py]   --- BRICK PROCESSOR READY ---
[assembly_plan.py]      --- ASSEMBLY ALLOCATOR READY ---
[assembly_plan.py]      Waiting for GUI requirements or camera supply...
```

> `Waiting for GUI requirements or camera supply...` printing every 500ms is **normal** — it means the bridge is running and waiting for the camera pipeline and a submitted plan.

<details>
<summary><b>What each bridge node does</b></summary>

**`firestore_bridge.py`**
Maintains a persistent listener on the Firebase Firestore `shapes` collection. Every time the operator submits or modifies the brick layout, this node receives the update and publishes raw JSON to `/incoming_bricks`.

**`brick_processor2.py`**
Subscribes to `/incoming_bricks`. Parses each brick's type (`L_shape`, `I_shape`, `T_shape`, `Z_shape`), orientation (`l_default`, `l_rotated`, `i_horizontal`, etc.), and grid position. Converts grid coordinates to physical world coordinates (cell size = 0.03m, Z height = 0.23m). Publishes to `/processed_bricks`.

**`assembly_plan.py`**
Validates the processed plan against camera-detected physical bricks (runs at 2Hz). Matches each required virtual brick to a physically detected brick. Assigns each brick to either ABB or AR4 based on workspace position. Exposes the final plan via the `get_assembly_plan` service that the supervisor calls.

</details>

---

## 2. Robot Environment

This single launch command starts everything robot-related: URDF model, MoveIt2 motion planning, RViz, ros2_control hardware interfaces, and all arm controllers.

### Scenario A — Simulation (Gazebo, both robots) ✅ Recommended for development

**Terminal 3**

```bash
source ~/collab_ws/install/setup.bash

# Both robots (default)
ros2 launch dual_arms master_launch.py

# ABB only
ros2 launch dual_arms master_launch.py mode:=sim robot:=abb

# AR4 only
ros2 launch dual_arms master_launch.py mode:=sim robot:=ar4
```

**Expected output** (all controllers active within ~30 seconds):
```
[gazebo]     Loaded world: dual.sdf
[spawner]    Configured and activated joint_state_broadcaster
[spawner]    Configured and activated ar4_trajectory_controller
[spawner]    Configured and activated irb120_trajectory_controller
[move_group] You can start planning now!
[rviz2]      Ready to take commands for planning group irb120_arm
[rviz2]      Ready to take commands for planning group ar_manipulator
```

---

### Scenario B — Real Hardware, ABB IRB120 Only

> ⚠️ **ABB PRE-LAUNCH CHECKLIST — complete before running:**
> 1. Power ON the IRC5 cabinet
> 2. Press the **white Motors On** button on the cabinet front panel
> 3. On FlexPendant: turn key to **Auto** mode
> 4. FlexPendant: **PP to Main** → press **▶ Play**
> 5. **RAPID must be running BEFORE launching ROS2**

**Terminal 3**

```bash
source ~/collab_ws/install/setup.bash

# Verify network first
ping 192.168.125.1

# Launch ABB real hardware
ros2 launch dual_arms master_launch.py mode:=real robot:=abb

# With custom IP
ros2 launch dual_arms master_launch.py mode:=real robot:=abb \
    abb_robot_ip:=192.168.125.1
```

```bash
# Verify EGM UDP packets are arriving (run in a separate terminal)
sudo tcpdump -i enp7s0 udp port 6511 -c 10
```

**Expected output:**
```
[rws_client]    RWS connection established
[rws_client]    Robot state: MOTORS_ON, execution_state: RUNNING
[ros2_control]  Successfully set initial state of ABBMultiInterface...
[spawner]       Configured and activated irb120_controller
[move_group]    You can start planning now!
```

---

### Scenario C — Real Hardware, AR4 Mk3 Only

```bash
source ~/collab_ws/install/setup.bash

# Check Teensy port
ls /dev/ttyACM*

# Launch with calibration (standard startup)
ros2 launch dual_arms master_launch.py mode:=real robot:=ar4 \
    ar4_serial_port:=/dev/ttyACM0 \
    ar4_calibrate:=True

# Skip calibration (after crash recovery — robot at known position)
ros2 launch dual_arms master_launch.py mode:=real robot:=ar4 \
    ar4_serial_port:=/dev/ttyACM0 \
    ar4_calibrate:=False
```

---

### Scenario D — Real Hardware, Both Robots

```bash
source ~/collab_ws/install/setup.bash

ros2 launch dual_arms master_launch.py mode:=real robot:=both \
    abb_robot_ip:=192.168.125.1 \
    ar4_serial_port:=/dev/ttyACM0 \
    ar4_calibrate:=True
```

---

### Launch Arguments Reference

| Argument | Default | Choices | Description |
|----------|---------|---------|-------------|
| `mode` | `sim` | `sim \| real` | Gazebo simulation or physical hardware |
| `robot` | `both` | `ar4 \| abb \| both` | Which robot(s) to launch |
| `ar4_calibrate` | `True` | `True \| False` | Run AR4 homing sequence on startup |
| `ar4_serial_port` | `/dev/ttyACM0` | any path | Teensy USB port for AR4 main controller |
| `ar4_arduino_port` | `/dev/ttyUSB0` | any path | Arduino port for AR4 gripper |
| `ar4_model` | `mk3` | `mk1 \| mk2 \| mk3` | AR4 hardware revision |
| `ar4_tf_prefix` | `ar4_` | any string | TF namespace prefix for AR4 links |
| `ar4_include_gripper` | `True` | `True \| False` | Include gripper in URDF |
| `abb_robot_ip` | `192.168.125.1` | any IP | ABB IRC5 controller IP address (RWS) |
| `abb_robot_port` | `80` | any port | ABB RWS HTTP port |

---

## 3. Detection & Grasping

Handles the camera stream, YOLO brick detection, and grasp point calculation. The launch command depends on whether the physical RealSense camera is connected.

### Scenario A — Real Hardware with Intel RealSense Camera

> ⚠️ **Prerequisite:** `sudo apt install ros-jazzy-realsense2-camera`

**Terminal 4**

```bash
source ~/collab_ws/install/setup.bash
ros2 launch detection_grasping_bringup hardware_bringup.launch.py
```

**What this launches:**
- `realsense2_camera` — RealSense driver with depth alignment (`align_depth.enable:=true`)
- `brick_detection/final_detector` — YOLO detection on `/camera/camera/color/image_raw`
- `brick_grasping_model/advanced_grasping_node.py` — grasp point from depth image
- `tf2_ros/static_transform_publisher` — `base_link` → `camera_color_optical_frame` at `[0.0546, -0.674, 0.769]`

**Camera parameters:**
```
static_z_height:  0.712        # calibrated table height in meters
image_topic:      /camera/camera/color/image_raw
depth_topic:      /camera/camera/aligned_depth_to_color/image_raw
camera_frame:     camera_color_optical_frame
depth_scale:      0.001        # RealSense depth units to meters
```

---

### Scenario B — Simulation with Virtual Camera

**Terminal 4**

```bash
source ~/collab_ws/install/setup.bash
ros2 launch detection_grasping_bringup software_bringup.launch.py
```

**What this launches:**
- `brick_detection/advanced_yolo` — YOLO detection on `/environment_camera/image_raw`
- `brick_grasping_model/advanced_grasping_node.py` — grasp from simulated depth

**Simulated camera parameters:**
```
static_z_height:  0.80
image_topic:      /environment_camera/image_raw
depth_topic:      /environment_camera/depth_image
camera_frame:     camera
depth_scale:      1.0          # Gazebo depth already in meters
```

---

### Scenario C — Combined Launch (Simulation)

```bash
source ~/collab_ws/install/setup.bash
ros2 launch detection_grasping_bringup detection_grasping_bringup.launch.py
```

```bash
# Verify detection is running
ros2 topic list | grep -E 'brick|grasp|detect'

# Check grasp service is available
ros2 service list | grep grasp

# Echo detected bricks (place bricks in camera view first)
ros2 topic echo /detected_bricks --once
```

---

## 4. Supervisor

The assembly state machine — launched **last**, after all other components are ready.

**Terminal 5**

```bash
source ~/collab_ws/install/setup.bash

# Simulation mode
ros2 run supervisor_package supervisor2 --ros-args -p use_sim:=true

# Real hardware mode
ros2 run supervisor_package supervisor2 --ros-args -p use_sim:=false
```

**Expected startup output:**
```
[supervisor] TF2 Static Broadcaster and Listener ready.
[supervisor] Supervisor Initialized. Waiting for services...
[supervisor] Requesting Assembly Plan from GUI...
[supervisor] Waiting for GUI node...
```

> The supervisor loops on `Waiting for GUI node...` until the GUI bridge is ready and a plan has been submitted. This is normal.

<details>
<summary><b>Supervisor State Machine</b></summary>

| State | What Happens | Next State |
|-------|-------------|------------|
| `INIT` | Calls `get_assembly_plan` from GUI bridge. Retries every 2s if empty. | `DETECT` |
| `DETECT` | Calls `detect_bricks`. Transforms brick poses from camera frame to ABB `base_link`. | `PROCESS_NEXT` |
| `PROCESS_NEXT` | Pops next brick from queue. If empty → `DONE`. | `GRASP_PIPELINE` |
| `GRASP_PIPELINE` | Calls `grasp/get_grasp_point`. Transforms grasp pose. Branches to ABB, AR4, or Handover. | `EXECUTE_ABB_PICK` or `EXECUTE_AR4_DIRECT` |
| `EXECUTE_ABB_PICK` | Sends `PICK` action to ABB. ABB moves to grasp and picks brick. | `EXECUTE_ABB_PLACE` |
| `EXECUTE_ABB_PLACE` | Computes placement via rigid body transform. ABB places brick. | `PROCESS_NEXT` |
| `EXECUTE_AR4_DIRECT` | AR4 approaches → visual servoing → grasp. | `AR4_PLACE_ON_GRID` |
| `AR4_PLACE_ON_GRID` | AR4 moves to placement pose and releases. | `PROCESS_NEXT` |
| `HANDOVER_SEQUENCE` | AR4 picks brick → moves to handover pose → ABB picks from handover → places. | `PROCESS_NEXT` |
| `RECOVERY` | Moves AR4 to home. Retries failed brick. If recovery fails → `EMERGENCY_STOP`. | `PROCESS_NEXT` |
| `DONE` | All bricks placed. Assembly complete. | — |

</details>

<details>
<summary><b>Services & Actions the Supervisor Depends On</b></summary>

| Service / Action | Type | Provided By |
|-----------------|------|-------------|
| `get_assembly_plan` | Service | `assembly_plan.py` (GUI bridge) |
| `detect_bricks` | Service | `brick_detection` / camera pipeline |
| `grasp/get_grasp_point` | Service | `brick_grasping_model` |
| `ar4_point_control` | Action | AR4 motion controller |
| `ar4_visual_servo` | Action | AR4 visual servoing node |
| `abb_control` | Action | ABB task server |
| `ar4_gripper/set` | Service | AR4 gripper node |

</details>

---

## Full System Checklists

### ✅ Checklist A — Full System in Simulation

```bash
# ═══════════════════════════════════════════════════════════
# TERMINAL 1 — GUI Web App
# ═══════════════════════════════════════════════════════════
cd ~/collab_ws/GUI_PACKAGE
npm install                       # first time only
npm start
# Press s then w → browser opens at http://localhost:8081
# Login: collabarms@gmail.com / collabarms@2025

# ═══════════════════════════════════════════════════════════
# TERMINAL 2 — Robot Environment (Gazebo Simulation)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch dual_arms master_launch.py
# Wait for: "You can start planning now!"

# ═══════════════════════════════════════════════════════════
# TERMINAL 3 — Detection & Grasping (Simulation)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch detection_grasping_bringup software_bringup.launch.py

# ═══════════════════════════════════════════════════════════
# TERMINAL 4 — GUI Bridge
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch ros2_gui_bridge gui_system.launch.py
# Wait for: "Connected! Listening to shapes..."
# Now design layout in browser and press Submit

# ═══════════════════════════════════════════════════════════
# TERMINAL 5 — Supervisor (launch LAST)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 run supervisor_package supervisor2 --ros-args -p use_sim:=true
```

---

### ✅ Checklist B — Full System with Real Hardware

> ⚠️ **Before starting:** Power ON IRC5 → Motors On → Auto mode → PP to Main → Play. RAPID must be running.

```bash
# ═══════════════════════════════════════════════════════════
# TERMINAL 1 — GUI Web App (same as simulation)
# ═══════════════════════════════════════════════════════════
cd ~/collab_ws/GUI_PACKAGE && npm start
# Press s then w
# Login: collabarms@gmail.com / collabarms@2025

# ═══════════════════════════════════════════════════════════
# TERMINAL 2 — Robot Environment (Real Hardware)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch dual_arms master_launch.py mode:=real robot:=both \
    abb_robot_ip:=192.168.125.1 \
    ar4_serial_port:=/dev/ttyACM0 \
    ar4_calibrate:=True
# Verify EGM: sudo tcpdump -i enp7s0 udp port 6511 -c 5

# ═══════════════════════════════════════════════════════════
# TERMINAL 3 — Detection & Grasping (Real RealSense)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch detection_grasping_bringup hardware_bringup.launch.py

# ═══════════════════════════════════════════════════════════
# TERMINAL 4 — GUI Bridge
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 launch ros2_gui_bridge gui_system.launch.py
# Design layout in browser → Submit

# ═══════════════════════════════════════════════════════════
# TERMINAL 5 — Supervisor (launch LAST)
# ═══════════════════════════════════════════════════════════
source ~/collab_ws/install/setup.bash
ros2 run supervisor_package supervisor2 --ros-args -p use_sim:=false
```

---

## Diagnostics & Verification

### After launching Robot Environment
```bash
# All controllers should be 'active'
ros2 control list_controllers

# Joint states should be publishing
ros2 topic hz /joint_states
# Expected: ~630Hz (sim) | ~250Hz (ABB EGM) | ~100Hz (AR4)

# Verify ABB RWS connection
ros2 service list | grep rws

# Verify EGM packets (real hardware only)
sudo tcpdump -i enp7s0 udp port 6511 -c 10
```

### After launching Detection & Grasping
```bash
# Check all expected topics
ros2 topic list | grep -E 'brick|grasp|detect|camera'

# Check grasp service is available
ros2 service list | grep grasp

# Echo detected bricks (place some bricks in camera view first)
ros2 topic echo /detected_bricks --once
```

### After launching GUI Bridge
```bash
# Check bridge topics
ros2 topic list | grep -E 'incoming|processed|brick'

# Echo processed bricks to verify coordinate conversion
ros2 topic echo /processed_bricks --once
```

### After launching Supervisor
```bash
# Supervisor node should appear
ros2 node list | grep supervisor

# If supervisor is stuck in INIT — check GUI bridge:
ros2 service call /get_assembly_plan supervisor_package/srv/GetAssemblyPlan '{}'

# If supervisor is stuck in DETECT — check camera service:
ros2 service call /detect_bricks dual_arms_msgs/srv/DetectBricks '{}'
```

### Emergency Stop & Recovery
```bash
# Software: Ctrl+C in the supervisor terminal
# Hardware: press red E-STOP button on IRC5 cabinet

# Restart supervisor after E-stop (without relaunching everything)
ros2 run supervisor_package supervisor2 --ros-args -p use_sim:=false

# If AR4 is in unknown position, relaunch with calibration:
ros2 launch dual_arms master_launch.py mode:=real robot:=ar4 \
    ar4_serial_port:=/dev/ttyACM0 ar4_calibrate:=True
```

> ⚠️ **After any E-stop on real hardware:** (1) Manually verify robots are in a safe position. (2) Check gripper state — it may be holding a brick. (3) Clear the workspace before restarting.

---

## Package Structure

```
collab_ws/src/
├── dual_arms_packages/
│   ├── dual_arms/                    # Main package: URDF, SRDF, launch files
│   ├── dual_arms_msgs/               # Custom ROS2 message definitions
│   ├── dual_arms_teach_panel/        # RViz2 Teach & Record panel plugin
│   ├── supervisor_package/           # Assembly state machine
│   ├── detection_grasping/           # Vision pipeline and grasp planning
│   └── ros2_gui_bridge/              # Firebase ↔ ROS2 bridge
├── abb_packages/
│   ├── abb_ros2/                     # ABB RWS client + EGM hardware interface
│   └── ros2_SimRealRobotControl/     # ros2srrc_execution motion primitives
└── ar4_packages/
    ├── annin_ar4_driver/             # AR4 Teensy serial hardware interface
    └── annin_ar4_description/        # AR4 URDF and mesh files

GUI_PACKAGE/                          # React Native / Expo web application
```

---

## Stored Programs

Assembly programs taught in the Teach & Record panel are saved to:

```
~/.ros/dt_programs/<program_name>.yaml
```

```bash
# List saved programs
ls ~/.ros/dt_programs/

# Execute a stored program on real hardware
ros2 topic pub /dt/program_execute std_msgs/String \
  '{data: "wall_assembly_v1"}' --once
```

---

*Dual-Arm Collaborative Robot System — ROS2 Jazzy · MoveIt2 · Gazebo Harmonic · March 2026*
