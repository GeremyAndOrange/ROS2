import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_x = LaunchConfiguration('x', default=0.)
    start_y = LaunchConfiguration('y', default=0.)
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
            {'use_sim_time': True}
        ]
    )

    spawn_node = Node(
        package = 'gazebo_ros', 
        executable = 'spawn_entity.py',
        arguments = [
            '-entity', robot_name,
            '-file', urdf_file,
            '-x', start_x,
            '-y', start_y
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

    cartographer_share = os.path.join(package_share, 'cartographer_config')
    cartohrapher_config = 'robot_config.lua'
    cartographer_node = Node(
        package = 'cartographer_ros',
        namespace = robot_name,
        executable = 'cartographer_node',
        name = 'cartographer',
        arguments = [
            '-configuration_directory', cartographer_share,
            '-configuration_basename', cartohrapher_config
        ],
        remappings = [
            ('submap_list','/submap_list')
        ]
    )
    
    launch_description.add_action(robot_node)
    launch_description.add_action(spawn_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(cartographer_node)
    # launch_description.add_action(occupancy_grid_node)

    return launch_description