from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Paths to other launch files ---
    dofbot_description_dir = get_package_share_directory('dofbot_description')
    dofbot_moveit_config_dir = get_package_share_directory('dofbot_moveit_config')
    dofbot_pick_place_dir = get_package_share_directory('dofbot_pick_place')
    dofbot_gesture_control_dir = get_package_share_directory('dofbot_gesture_control')

    # --- Launch MoveIt + Gazebo ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dofbot_description_dir, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # --- Launch C++ pick_place_test node ---
    pick_place_node = Node(
        package='dofbot_gesture_control',
        executable='hand_follower',
        output='screen'
    )

    # --- Launch Python gesture control node ---
    gesture_node = Node(
        package='dofbot_gesture_control',
        executable='gesture_control_node.py',
        output='screen'
    )

    # --- Launch description ---
    return LaunchDescription([
        moveit_launch,
        pick_place_node,
        gesture_node
    ])
