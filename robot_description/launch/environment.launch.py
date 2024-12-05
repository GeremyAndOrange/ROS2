import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui_open = LaunchConfiguration('gui_open', default='false')
    rviz_open = LaunchConfiguration('rviz_open', default='true')
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

    gazebo_world_path = os.path.join(package_share, 'world/hard.world')
    start_gzserver_cmd = ExecuteProcess(
        cmd = [
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            gazebo_world_path
        ],
        condition=UnlessCondition(gui_open)
    )
    start_gazebo_cmd = ExecuteProcess(
        cmd = [
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            gazebo_world_path
        ],
        condition=IfCondition(gui_open)
    )

    global_map_node = Node(
        package = 'manager',
        executable = 'global_map_node',
        name = 'global_map_node',
        parameters = [
            {'use_sim_time': True}
        ]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_open),
        arguments=[
            '--ros-args', 
            '--log-level', 'warn'
        ]
    )

    launch_description.add_action(manager_node)
    launch_description.add_action(start_gzserver_cmd)
    launch_description.add_action(start_gazebo_cmd)
    launch_description.add_action(global_map_node)
    launch_description.add_action(rviz2_node)

    return launch_description