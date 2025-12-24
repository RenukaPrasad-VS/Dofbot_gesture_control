# 🤖  DOFBOT – ROS 2 Vision-Based Pick & Place

This repository contains a complete **ROS 2 (Humble)** software stack for the **Yahboom DOFBOT 6-DOF robotic arm**, supporting **simulation and real-hardware execution**, **MoveIt 2 motion planning**, and **vision-based pick and place** using an **Intel RealSense RGB-D camera**.

The project is structured for **clean architecture**, **real-robot reliability**, and **interview/demo readiness**.

---

## 📌 Key Features

- ROS 2 Humble compatible
- Modular ROS 2 package structure
- URDF/Xacro-based robot description
- MoveIt 2 motion planning
- `ros2_control` hardware + simulation support
- Intel RealSense RGB + depth integration
- OpenCV-only HSV color detection
- PointCloud2 → TF → grasp target pipeline
- Cartesian and joint-space motion execution
- End-to-end pick-and-place demo

---

## 📁 Repository Structure

dofbot_ws/src
├── dofbot_description # URDF, Xacro, meshes, TF

├── dofbot_moveit_config # MoveIt 2 configuration

├── dofbot_hw # ros2_control hardware interface

├── dofbot_vision # RealSense + OpenCV perception

├── dofbot_pick_place # Motion execution & pick-place logic

├── dofbot_gesture_control # Gesture-based control (optional)

├── rs_pointcloud # Custom pointcloud processing


---

## Prerequisites

### System
  - Ubuntu 22.04
  - ROS 2 Humble **|** [Humble](https://docs.ros.org/en/humble/Installation.html)
  - Ignition Gazebo **|** [Fortress](https://gazebosim.org/docs/fortress) (default version in ROS 2 Humble)


## ROS 2 Dependencies
```
sudo apt update
sudo apt install -y \
  ros-humble-ros-ign \
  ros-humble-ros-ign-bridge \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-image \
  ros-humble-ros-ign-gazebo-demos
```
## Workspace Setup
```
mkdir -p ~/dofbot_ws/src
cd ~/dofbot_ws/src
git clone https://github.com/God-Official/PickPlace_6Dof-Robotic_Arm.git
cd ..
```

## Build Workspace
```
colcon build
source install/setup.bash
```

## Launching the Simulation
**1️⃣ Visualize Robot in RViz**
```
ros2 launch dofbot_description display.launch.py
```

**2️⃣ Launch Ignition Gazebo with DOFBOT**
```
ros2 launch dofbot_description gazebo.launch.py
```
This will:
- Start Ignition Gazebo Fortress
- Spawn the DOFBOT robot
- Load ros2_control controllers

## MoveIt 2 Motion Planning
**Launch MoveIt Demo**
```
ros2 launch dofbot_moveit_config demo.launch.py
```

This launches:
- MoveIt move_group
- RViz with motion planning plugin
- DOFBOT kinematics & controllers

You can:
- Plan joint-space motions
- Execute trajectories
- Test end-effector goals

## Launching the Simulation with Moveit 2
```
ros2 launch dofbot_description moveit.launch.py
```
This will:
- Start Ignition Gazebo Fortress
- Spawn the DOFBOT robot
- Load ros2_control controllers
- Moveit with rviz

## Connecting Real Hardware   (Optional - Only for Real Bot)
```
ros2 run dofbot_hw interface.py
```

## RealSense PointCloud (Optional - Only for Real Bot)
```
ros2 run rs_pointcloud rs_pointcloud.py
```
Publishes pointcloud data from RealSense camera (real hardware).

## Pick & Place Pipeline
**1️⃣ TF Broadcasting**
```
ros2 run dofbot_vision block_tf.py #(real_bot)
ros2 run dofbot_vision block_detect.py #(for sim)
```
Converts detected object positions into TF frames.

**2️⃣ Pick & Place Execution**
```
ros2 run dofbot_pick_place pick_place_node
```
This node:
- Receives object pose
- Plans pick trajectory
- Executes grasp and place sequence

## Tested Configuration
- ROS 2 Humble
- Ignition Gazebo Fortress
- MoveIt 2
- Ubuntu 22.04
- Python 3.10
- C++17

## Credit & Contributions

**Contributors:**
- Prateek H A **|** [Github](https://github.com/God-Official)
- Renuka Prasad VS **|** [Github](https://github.com/RenukaPrasad-VS)

**Contributions:**
- The complete system design, development, and integration of this project were carried out by the contributors listed above.
- All ROS 2 integration, MoveIt 2 motion planning, simulation setup, perception pipeline, and pick & place logic were jointly developed.


## Professional Context

- This project was developed during our employment at a company.  
- The repository contains only non-confidential and non-proprietary components and is shared for technical demonstration purposes.



