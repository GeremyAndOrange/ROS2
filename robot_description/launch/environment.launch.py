import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui_open = LaunchConfiguration('gui_open', default='false')
    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)

    manager_node = Node(
        package = 'manager',
        executable = 'manager_node',
        name = 'manager_node',
        parameters = [
            {'use_sim_time': True}
        ]
    )

    gazebo_command = 'gzserver' if gui_open == 'false' else 'gazebo'
    gazebo_world_path = os.path.join(package_share, 'world/hard.world')
    start_gazebo_cmd = ExecuteProcess(
        cmd = [
            gazebo_command,
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
             gazebo_world_path
        ],
    )

    global_map_node = Node(
        package = 'manager',
        executable = 'global_map_node',
        name = 'global_map_node',
        parameters = [
            {'use_sim_time': True}
        ]
    )

    launch_description.add_action(manager_node)
    launch_description.add_action(start_gazebo_cmd)
    launch_description.add_action(global_map_node)

    return launch_description