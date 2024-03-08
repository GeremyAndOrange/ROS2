import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'robot_description'

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 

    # Start Gazebo server
    gazebo_world_path = os.path.join(pkg_share, 'world/hard.world')
    start_gazebo_cmd = ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_path])
    # gazebo --verbose -s libgazebo_ros_factory.so src/robot_description/world/hard.world

    # Start Manager Node
    manager_node = Node(
        package='manager',
        executable='manager_node',
        name='manager_node'
        )

    # Start RVIZ2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(manager_node)
    ld.add_action(rviz2)

    return ld