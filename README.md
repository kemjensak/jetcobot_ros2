# JetCobot - ROS2 Robot Arm Control System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)

JetCobot is a comprehensive ROS2 package suite for controlling a 6-DOF robotic arm with gripper functionality, camera integration, and advanced motion planning capabilities using MoveIt2.

## 🚀 Features

- **6-DOF Robotic Arm Control**: Full kinematic control of a 6-axis robotic arm
- **MoveIt2 Integration**: Advanced motion planning and trajectory execution
- **Real-time Joint Control**: Seamless switching between simulated and real robot states
- **Gripper Control**: Integrated gripper manipulation with feedback
- **Camera Integration**: Multi-camera support with calibrated camera info publishing
- **Predefined Poses**: Ready-to-use poses for common operations (home, ready_to_grab, scan positions)
- **URDF/SRDF Models**: Complete robot description with visual meshes and collision models
- **Launch Files**: Comprehensive launch configurations for different use cases

## 📁 Package Structure

```
jetcobot/
├── jetcobot_bringup/          # Main control and launch package
│   ├── jetcobot_bringup/      # Python modules
│   │   ├── joint_control.py   # Real robot joint controller
│   │   ├── joint_state_switcher.py  # State switching logic
│   │   └── camera_info_publisher.py # Camera calibration publisher
│   ├── launch/                # Launch files
│   ├── config/                # Configuration and calibration files
│   └── package.xml
├── jetcobot_description/      # Robot model and visualization
│   ├── urdf/                  # URDF robot descriptions
│   ├── meshes/                # 3D mesh files (STL/DAE)
│   ├── launch/                # Display launch files
│   └── package.xml
└── jetcobot_moveit_config/    # MoveIt2 configuration
    ├── config/                # MoveIt2 configuration files
    ├── launch/                # MoveIt2 launch files
    └── package.xml
```

## 🛠️ Dependencies

### System Requirements
- Ubuntu 24.04 LTS (recommended)
- ROS2 Jazzy
- Python 3.8+

### Python Dependencies
```bash
pip install pymycobot packaging
```

## 🚀 Installation

1. **Create a ROS2 workspace** (if you don't have one):
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```

2. **Clone the repository**:
```bash
git clone https://github.com/kemjensak/jetcobot.git
```

3. **Install dependencies**:
```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y # or 'rd'
```

4. **Build the workspace**:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install # or 'cb'
source ./install/local_setup.bash # or 'sb'
```

## 🎮 Usage

### Basic Robot Control

#### Distributed Control Setup (Recommended)
For optimal performance, run the system in a distributed manner:

**On the Raspberry Pi (connected to JetCobot):**
```bash
# Run the robot hardware interface and control nodes
ros2 launch jetcobot_bringup bringup.launch.py
```

**On the Control Computer (for planning and visualization):**
```bash
# Launch MoveIt2 with RViz for motion planning and control
ros2 launch jetcobot_moveit_config moveit_rviz.launch.py
```

> **Note**: Ensure both machines are on the same network and can communicate via ROS2. You may need to configure ROS2 networking settings such as `ROS_DOMAIN_ID` and firewall rules.



### Gripper Control

Control the gripper using ROS2 topics:
```bash
# Open gripper (value: 0-100, 0 = fully open)
ros2 topic pub /gripper_command std_msgs/msg/Int32 "data: 0"

# Close gripper
ros2 topic pub /gripper_command std_msgs/msg/Int32 "data: 100"

# Check gripper feedback
ros2 topic echo /gripper_feedback
```

### Predefined Poses

The robot comes with several predefined poses that can be used with MoveIt2:

- **home**: Default home position
- **ready_to_see**: Optimal position for camera operations
- **ready_to_grab**: Position for grasping operations
- **scan_front**: Front scanning position
- **scan_left**: Left scanning position
- **scan_right**: Right scanning position

### Camera Integration

The system supports multiple cameras with calibration:
```bash
# Launch camera info publisher
ros2 launch jetcobot_bringup camera_info_publisher.launch.py camera_name:=ov3360 frame_id:=ov3360
```

Supported camera configurations:
- `jetcocam_1_calib.yaml` / `jetcocam_2_calib.yaml`
- `ov3360_1_calib.yaml` / `ov3360_2_calib.yaml` / `ov3360_3_calib.yaml`

## 🔧 Configuration

### Joint Limits
Joint limits are configured in `jetcobot_moveit_config/config/joint_limits.yaml`:
- Position limits for all 6 joints
- Velocity and acceleration limits
- Safety margins

### Controller Configuration
Robot controllers are defined in:
- `jetcobot_moveit_config/config/ros2_controllers.yaml`
- `jetcobot_moveit_config/config/moveit_controllers.yaml`

### Camera Calibration
Camera calibration files are stored in `jetcobot_bringup/config/`:
- Intrinsic parameters
- Distortion coefficients
- Image dimensions

## 🏗️ Architecture

### Node Communication Flow

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│  joint_control  │◄──►│joint_state_      │◄──►│     MoveIt2         │
│     _node       │    │   switcher       │    │  move_group        │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│ Real Hardware   │    │  Joint States    │    │ Trajectory          │
│   (MyCobot)     │    │   /joint_states  │    │    Execution        │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
```

### Key Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions |
| `/real_joint_states` | `sensor_msgs/JointState` | Real hardware joint states |
| `/gripper_command` | `std_msgs/Int32` | Gripper position command |
| `/gripper_feedback` | `std_msgs/Int32` | Gripper position feedback |
| `/get_angles_cmd` | `std_msgs/Bool` | Request current joint angles |

### Services and Actions

- **MoveIt2 Actions**: `/arm_group_controller/follow_joint_trajectory`
- **Move Group Planning**: Standard MoveIt2 planning services
- **Controller Management**: ROS2 controller lifecycle services

## 🤖 Hardware Integration

This package is designed to work with MyCobot280 robot arm:

1. **Connection**: Ensure proper USB/Serial connection to the robot
2. **Permissions**: Grant appropriate permissions to the serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```
3. **Firmware**: Ensure the robot firmware is compatible with pymycobot

## 🐛 Troubleshooting

### Common Issues

1. **Robot not responding**:
   - Check USB connection and permissions
   - Verify pymycobot installation
   - Check if robot is in proper mode

2. **MoveIt2 planning failures**:
   - Verify joint limits configuration
   - Check for collisions in the planning scene
   - Ensure start state is valid

3. **Camera calibration issues**:
   - Verify camera calibration file paths
   - Check camera permissions and drivers

### Debug Commands

```bash
# Check joint states
ros2 topic echo /joint_states

# Monitor controller status
ros2 control list_controllers

# Check transform tree
ros2 run tf2_tools view_frames

# Monitor MoveIt2 planning
ros2 topic echo /move_group/display_planned_path
```


## 👥 Maintainers

- **Jinseok Kim** - *Lead Developer* - [jinseok.kim970@gmail.com](mailto:jinseok.kim970@gmail.com)

## 🙏 Acknowledgments

- [MoveIt2](https://moveit.ros.org/) - Motion planning framework
- [ROS2](https://docs.ros.org/en/humble/) - Robot Operating System
- [MyCobot](https://www.elephantrobotics.com/) - Robot hardware platform
- The ROS community for their invaluable contributions

## 📞 Support

For support and questions:
- 📧 Email: [jinseok.kim970@gmail.com](mailto:jinseok.kim970@gmail.com)
- 🐛 Issues: [GitHub Issues](../../issues)
- 💬 Discussions: [GitHub Discussions](../../discussions)

---

*Made with ❤️ by the team BOLT*
