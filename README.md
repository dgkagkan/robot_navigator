# 🤖 Prototype — Autonomous Robot Car

A ROS 2 Jazzy autonomous robot car with SLAM mapping, lifecycle-based action controller, and obstacle-aware path planning. Built with a custom URDF, simulated in Gazebo, and visualized in RViz2.

---

## 📽️ Demo

> (https://youtu.be/KyvREW2cXuE)

---

## ✨ Features

- **Custom URDF** — Robot model built from scratch with LiDAR and camera sensors
- **SLAM Mapping** — Real-time environment mapping using LiDAR
- **Lifecycle Action Controller** — C++ action server/client with full ROS 2 lifecycle management
- **Obstacle-Aware Navigation** — Python API sends planned paths to avoid obstacles
- **Goal-based Keyboard Controller** — Each keypress sends an incremental navigation goal to the action server, leveraging the full nav stack for automatic obstacle avoidance
- **Gazebo + RViz2** — Full simulation and visualization pipeline

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────┐
│                   ROS 2 Jazzy                   │
│                                                 │
│  [Keyboard Controller]                          │
│       │                                         │
│  (Goal Coordinates)                             │
│       │                                         │
│  [nav_bridge_py] ──► [Nav2 Action Server]       │
│                              │                  │
│                      (Path Planning +           │
│                      Obstacle Avoidance)        │
│                              │                  │
│                     [Action Server]             │
│                     (Lifecycle Node C++)        │
│                              │                  │
│                       [Robot URDF]              │
│                    LiDAR │   Camera             │
│                          │                      │
│                   [SLAM Mapper]                 │
│                          │                      │
│                      /map topic                 │
│                          │                      │
│                       [RViz2]                   │
└─────────────────────────────────────────────────┘
```

---

## 📦 Package Structure

```
robot_navigator/
└── src/
    ├── my_robot_description/      # Custom URDF + RViz config
    │   ├── urdf/
    │   └── rviz/
    ├── my_robot_bringup/          # Launch files + config
    │   ├── launch/
    │   │   ├── my_robot_gazebo.launch.xml
    │   │   ├── navigation.launch.xml
    │   │   └── bringup_all.launch.py
    │   ├── config/
    │   │   ├── nav2_params_custom.yaml
    │   │   └── gazebo_bridge.yaml
    │   └── worlds/
    ├── my_robot_interfaces/       # Custom ROS 2 interfaces
    │   ├── action/
    │   └── msg/
    ├── robot_controller_cpp/      # Lifecycle action server (C++)
    │   ├── include/
    │   └── src/
    ├── robot_controller_nav2/     # Goal-based keyboard controller (C++)
    │   └── src/
    └── nav_bridge_py/             # Navigation bridge (Python)
```

---

## 🚀 Getting Started

### Prerequisites

- ROS 2 Jazzy
- Gazebo
- `slam_toolbox`
- `nav2_bringup`
## ⚠️ Before Running

Copy the required files to your home directory:

```bash
# Map files
mkdir -p ~/maps
cp src/maps/cubes.pgm ~/maps/
cp src/maps/cubes.yaml ~/maps/

# Nav2 params
cp src/my_robot_bringup/config/nav2_params_custom.yaml ~/
```

> Note: Nav2 params are tuned for low-end hardware. Modify if needed.

### Build

```bash
git clone https://github.com/dgkagkan/robot_navigator.git
cd robot_navigator
colcon build
source install/setup.bash
```

### Run

**1. Start simulation (Gazebo + RViz2)**
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

**2. Start navigation (SLAM + Nav2)**
```bash
ros2 launch my_robot_bringup navigation.launch.xml
```

**3. Or launch everything at once**
```bash
ros2 launch my_robot_bringup bringup_all.launch.py
```

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Framework | ROS 2 Jazzy |
| Language | C++ / Python |
| Simulation | Gazebo |
| Visualization | RViz2 |
| Mapping | SLAM Toolbox |
| Controller | Lifecycle Action Server |
| Sensors | LiDAR, Camera (URDF) |

---

## 🔭 Future Work

- Integrate camera for visual object detection (YOLO)
- Dynamic obstacle avoidance with real-time map updates
- Web UI for remote control (Flask + WebSocket)
- Deploy on physical hardware

---

## 📄 License

MIT License
