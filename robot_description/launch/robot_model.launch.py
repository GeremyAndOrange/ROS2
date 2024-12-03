import os
from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_description = LaunchDescription()

    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)
    urdf_file = os.path.join(package_share, 'urdf/robot.urdf')
    xacro_file = os.path.join(package_share, 'urdf/robot.urdf.xacro')

    xacro_command = ExecuteProcess(
        cmd = [
            'xacro', xacro_file,
            PythonExpression(['str(','"robot_name:=robot")']),
            '-o', urdf_file,
        ]
    )

    python_script = os.path.join(package_share, 'cartographer_config/cartographer_config.py')
    cartographer_src = os.path.join(package_share, 'cartographer_config/robot_config.lua')
    cartographer_dst = os.path.join(package_share, 'cartographer_config/use_robot_config.lua')
    config_command = ExecuteProcess(
        cmd = [
            'python3', python_script, 'robot', cartographer_src, cartographer_dst
        ]
    )

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

    spawn_node = Node(
        package = 'gazebo_ros', 
        executable = 'spawn_entity.py',
        arguments = [
            '-entity', 'robot',
            '-file', urdf_file,
            '-x', "0.",
            '-y', "2."
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace = 'robot',
        arguments=[urdf_file]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace = 'robot',
        name='joint_state_publisher',
        arguments=[urdf_file]
        )

    cartographer_share = os.path.join(package_share, 'cartographer_config')
    cartohrapher_config = 'use_robot_config.lua'
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        namespace = 'robot',
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
        executable = 'cartographer_occupancy_grid_node',
        namespace = 'robot',
        name = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': True}
        ]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    launch_description.add_action(xacro_command)
    launch_description.add_action(config_command)
    launch_description.add_action(start_gazebo_cmd)
    launch_description.add_action(spawn_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(joint_state_publisher_node)
    launch_description.add_action(cartographer_node)
    launch_description.add_action(occupancy_grid_node)
    launch_description.add_action(rviz2_node)

    return launch_description