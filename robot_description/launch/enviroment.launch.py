import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_world_path = os.path.join(package_share, 'world/hard.world')
    start_gazebo_cmd = ExecuteProcess(
        cmd = [
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
             gazebo_world_path
        ],
    )

    launch_description.add_action(start_gazebo_cmd)

    return launch_description