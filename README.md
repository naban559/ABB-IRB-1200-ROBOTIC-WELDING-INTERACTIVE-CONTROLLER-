# ABB IRB 1200 – Standalone Robotic Welding Simulation

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition_Fortress-orange.svg)](https://gazebosim.org/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-green.svg)](https://moveit.picknik.ai/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

A **100% self-contained ROS 2 Humble application package** that simulates automated industrial robotic welding for the **ABB IRB 1200** 6-axis robot.

> **Key Feature:** This package requires **NO external ABB source packages** in your ROS 2 workspace. All robot kinematics, 3D meshes (visual & collision), MoveIt 2 configurations, `ros2_control` hardware interfaces, Gazebo SDF environments, RGB-D vision guidance, and Cartesian welding algorithms are embedded directly inside this single package.

---

## 🏗️ System Overview & Architecture

```
                  ┌─────────────────────────────────────────┐
                  │      Terminal Keyboard Controller       │
                  │ (S: Straight, L: L-shape, M: Multi-pt,  │
                  │  C: Camera Vision, H: Home, X: E-Stop)  │
                  └────────────────────┬────────────────────┘
                                       │ ROS 2 Action / Service
                                       ▼
                  ┌─────────────────────────────────────────┐
                  │        Welding Controller Node          │
                  │  (8-Stage State Machine + Interlocks)   │
                  └─────────┬─────────────────────┬─────────┘
                            │                     │
       Cartesian Trajectory │                     │ Camera Stream
                            ▼                     ▼
              ┌────────────────────────┐  ┌────────────────────────┐
              │        MoveIt 2        │  │ Overhead RGB-D Camera  │
              │  (OMPL + KDL Kinematics│  │  (OpenCV Seam Feature  │
              │   + 20 Hz Retiming)    │  │       Detection)       │
              └─────────────┬──────────┘  └────────────────────────┘
                            │ Joint Trajectory
                            ▼
              ┌────────────────────────┐
              │  Gazebo Sim (Ignition) │
              │ (Physics + Controllers │
              │  + Electric Arc Sparks)│
              └────────────────────────┘
```

---

## 📦 Package Layout

```
abb_welding_standalone/
├── CMakeLists.txt                      # Build configuration
├── package.xml                         # ROS 2 Humble dependencies
├── LICENSE                             # Apache-2.0 open-source license
├── NOTICE                              # Upstream ABB & ROS-Industrial attributions
├── README.md                           # Documentation & quick start guide
├── abb_welding_standalone/             # Python package
│   ├── __init__.py
│   ├── welding_controller_node.py      # Core Cartesian controller & state machine
│   ├── welding_visualizer_node.py      # Live sparks, arc glow, and bead markers
│   ├── welding_keyboard.py             # Interactive terminal keyboard controller
│   ├── seam_detector.py                # OpenCV RGB-D perception & TF transformation
│   ├── trajectory_utils.py             # Analytical trajectory generators & math
│   └── welding_mission_manager.py      # CLI execution utility
├── config/                             # Motion planning & controller configurations
│   ├── abb_irb1200_welding.srdf.xacro  # MoveIt 2 semantic robot description
│   ├── kinematics.yaml                 # KDL kinematics plugin setup
│   ├── joint_limits.yaml               # IRB 1200 velocity & acceleration limits
│   ├── ompl_planning.yaml              # OMPL motion planning pipeline
│   ├── moveit_controllers.yaml         # MoveIt trajectory execution controllers
│   ├── gazebo_controllers.yaml         # Gazebo ros2_control configuration
│   ├── ros2_controllers.yaml           # Mock hardware controller parameters
│   └── welding_params.yaml             # Certified welding & seam parameters
├── urdf/                               # Workcell URDF / Xacro models
│   ├── common_colours.xacro            # Standard RAL color definitions
│   ├── common_materials.xacro          # Material macros
│   ├── irb1200_5_90_macro.xacro        # Self-contained IRB 1200 link & joint model
│   ├── irb1200.ros2_control.xacro      # Embedded ros2_control system interface
│   ├── welding_torch.xacro             # MIG/GMAW welding torch with TCP definition
│   ├── welding_table.xacro             # Heavy-duty industrial welding table
│   ├── workpiece.xacro                 # V-groove chamfered steel plates with seam frames
│   ├── welding_camera.xacro            # Overhead RGB-D sensor housing & optics
│   └── welding_workcell.xacro          # Master integrated workcell model
├── meshes/                             # Embedded 3D CAD meshes
│   ├── collision/                      # Collision hulls for table, torch, and plates
│   ├── visual/                         # Visual meshes for table, torch, and plates
│   └── irb1200_5_90/                   # Full visual & collision meshes for IRB 1200
├── launch/                             # Launch automation scripts
│   ├── gazebo_welding.launch.py        # Master launch: Gazebo + MoveIt + RViz + Nodes
│   ├── welding.launch.py               # RViz-only fake hardware launcher
│   └── gazebo.launch.py                # Minimal Gazebo physics test launcher
├── rviz/
│   └── welding_sim.rviz                # Clean industrial RViz 2 layout
├── scripts/                            # Executable node binaries
└── worlds/
    └── welding_workcell.sdf            # Gazebo Ignition simulation world
```

---

## 🛠️ System Prerequisites & Dependencies

### Operating System & ROS 2
- **Ubuntu 22.04 LTS (Jammy Jellyfish)**
- **ROS 2 Humble Hawksbill**
- **Ignition Gazebo Fortress (`ign-gazebo6` / `ros-humble-ros-gz-sim`)**

### Install System Dependencies via apt:
```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-controller-manager \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-numpy
```

---

## 🚀 Quick Start (Clean Workspace Setup)

You can place `abb_welding_standalone` into **any** clean ROS 2 Humble workspace:

```bash
# 1. Create a workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 2. Copy or clone this package into src/
# cp -r /path/to/abb_welding_standalone ~/robot_ws/src/

# 3. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 4. Build the package
cd ~/robot_ws
colcon build --packages-select abb_welding_standalone

# 5. Source the workspace installation
source install/setup.bash
```

---

## 🎮 Launching the Simulation

### 1. Launch Master Gazebo Ignition Simulation
In Terminal 1:
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch abb_welding_standalone gazebo_welding.launch.py
```
*Gazebo Ignition Fortress and RViz 2 will open. The robot arm, MIG welding torch, industrial table, V-groove workpieces, and overhead RGB-D camera are loaded automatically.*

### 2. Launch Interactive Terminal Keyboard Controller
In Terminal 2:
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run abb_welding_standalone welding_keyboard
```

### 3. Keyboard Commands Reference
| Key | Command | Description |
|:---:|:---|:---|
| **`S`** | **Straight-Line Welding** | Executes certified 8-stage longitudinal seam welding ($180\text{ mm}$, travel speed $15\text{ mm/s}$). |
| **`L`** | **L-Shaped Corner Welding** | Executes $C^1$-continuous fillet corner seam welding ($213\text{ mm}$). |
| **`M`** | **Multi-Waypoint Welding** | Executes sinusoidal multi-waypoint weaving seam welding. |
| **`C`** | **Camera-Based Autonomous Welding** | Detects workpiece using overhead RGB-D camera + OpenCV, extracts seam coordinates via TF, verifies MoveIt reachability, and welds without hardcoded coordinates. |
| **`H`** | **Safe Return Home** | Returns manipulator to safe standby posture. Interlocked during active arc welding. |
| **`X`** | **Emergency Stop** | Instantly aborts active trajectory execution and extinguishes electric arc. |
| **`Q`** | **Quit Interface** | Safely exits the keyboard interface. |

---

## 👁️ Autonomous Vision Guidance (Displaced Workpiece Test)

To verify that camera mode calculates the seam dynamically from computer vision:

Launch Gazebo with the workpiece offset by $+20\text{ mm}$ in $X$ and $+40\text{ mm}$ in $Y$:
```bash
ros2 launch abb_welding_standalone gazebo_welding.launch.py auto_start:=false workpiece_x:=0.02 workpiece_y:=0.04
```

In the keyboard terminal, press **`C`**.  
The camera detects the displaced workpiece, transforms coordinates into the robot base frame, plans reachability-validated Cartesian waypoints, and completes the weld with sub-millimeter precision ($\le 0.45\text{ mm}$ error).

---

## 📜 License & Acknowledgements

This package is licensed under the **Apache License 2.0**. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE) for complete details.

Includes CAD models and kinematics derived from the **ROS-Industrial Consortium** and **ABB Robotics** (`abb_ros2` repository).

