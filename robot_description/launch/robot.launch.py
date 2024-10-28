import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    origin_x = LaunchConfiguration('origin_x', default=0.)
    origin_y = LaunchConfiguration('origin_y', default=0.)
    robot_name = LaunchConfiguration('robot_name', default='robot')

    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)
    urdf_file = os.path.join(package_share, 'urdf/robot.urdf')

    robot_node = Node(
        package = 'robot_control',
        namespace = robot_name,
        executable='robot_node',
        name = 'control',
        parameters = [
            {'name': robot_name},
            {'use_sim_time': True},
            {'origin_x': origin_x},
            {'origin_y': origin_y}
        ]
    )

    spawn_node = Node(
        package = 'gazebo_ros', 
        executable = 'spawn_entity.py',
        arguments = [
            '-entity', robot_name,
            '-file', urdf_file,
            '-x', origin_x,
            '-y', origin_y
        ]
    )

    with open(urdf_file,'r') as inf:
        robot_description = inf.read()
    robot_state_publisher_node =  Node(
        package = 'robot_state_publisher',
        namespace = robot_name,
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        namespace = robot_name,
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters = [
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    cartographer_share = os.path.join(package_share, 'cartographer_config')
    cartohrapher_config = 'use_robot_config.lua'
    cartographer_node = Node(
        package = 'cartographer_ros',
        namespace = robot_name,
        executable = 'cartographer_node',
        name = 'cartographer',
        arguments = [
            '-configuration_directory', cartographer_share,
            '-configuration_basename', cartohrapher_config
        ],
        parameters = [
            {'use_sim_time': True}
        ]
    )
    
    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        namespace = robot_name,
        executable = 'cartographer_occupancy_grid_node',
        name = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': True}
        ],
        remappings = [
            ('map','/submap')
        ]
    )

    launch_description.add_action(robot_node)
    launch_description.add_action(spawn_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(joint_state_publisher_node)
    launch_description.add_action(cartographer_node)
    launch_description.add_action(occupancy_grid_node)

    return launch_description