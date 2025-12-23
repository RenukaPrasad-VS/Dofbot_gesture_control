# 🤖 Yahboom DOFBOT – ROS 2 Vision-Based Pick & Place

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

## 🖥️ System Requirements

### Hardware
- Yahboom DOFBOT (6-DOF)
- Jetson Nano / Jetson Orin / Ubuntu PC
- Intel RealSense camera (D435 / D455)

### Software
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- OpenCV
- Intel RealSense SDK (`librealsense2`)

---

## 📦 Installation

### 1️⃣ Clone the Repository
```bash
cd ~/dofbot_ws/src
https://github.com/RenukaPrasad-VS/Dofbot_gesture_control.git


##Build the Workspace

colcon build --symlink-install
source install/setup.bash



##🧪 Simulation (Gazebo / Ignition)

Launch the DOFBOT simulation and moveit:

ros2 launch dofbot_description moveit.launch.py

##For color block detection
ros2 run dofbot_vision block_tf.py

## For pont cloud 
ros2 run rs_pointcloud rs_pointcloud.py

## For pick and place 
ros2 run dofbot_pick_place pick_place_node



## 🤝 Contributors

- Renuka Prasad – Core development, ROS 2, MoveIt, vision pipeline  
- Prateek H A – Core development, ROS 2, MoveIt, vision pipeline 


