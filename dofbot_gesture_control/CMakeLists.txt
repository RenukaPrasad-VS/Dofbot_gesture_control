cmake_minimum_required(VERSION 3.8)
project(dofbot_gesture_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

install(
  PROGRAMS
    scripts/gesture_control_node.py
    scripts/dofbot_hand_follower.py
    scripts/hand_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(hand_follower src/hand_follower.cpp)
ament_target_dependencies(hand_follower
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS hand_follower
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch src scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
