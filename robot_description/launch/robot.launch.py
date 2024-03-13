import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_x = LaunchConfiguration('x', default=0.)
    start_y = LaunchConfiguration('y', default=0.)
    robot_id = LaunchConfiguration('robot_id', default='0')
    robot_name = LaunchConfiguration('robot_name', default='robot')

    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)

    robot_node = Node(
        package = 'robot_control',
        namespace = robot_name,
        executable='robot_node',
        name = 'control',
        parameters = [{'id': robot_id}]
    )

    xacro_file = os.path.join(package_share, 'urdf/robot.urdf.xacro')
    urdf_file = os.path.join(package_share, 'urdf/robot.urdf')
    xacro_command = ExecuteProcess(
        cmd = [
            'xacro',xacro_file,
            PythonExpression(['str(','"robot_name:=',robot_name,'")']),
            '-o',urdf_file,
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

    # joint_state_publisher_node = Node(
    #     package = 'joint_state_publisher_gui',
    #     namespace = robot_name,
    #     executable = 'joint_state_publisher_gui',
    #     name = 'joint_state_publisher_gui',
    #     arguments = [urdf_file]
    #     )

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = robot_name,
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        arguments = [urdf_file]
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

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_persec = LaunchConfiguration('publish_persec', default='1.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'occupancy_grid_node',
        name = 'occupancy_grid_node',
        parameters = [{'use_sim_time': use_sim_time}],
        arguments = [
            '-resolution', resolution,
            '-publish_period_sec', publish_persec
        ]
    )

    launch_description.add_action(robot_node)
    launch_description.add_action(xacro_command)
    # launch_description.add_action(spawn_node)
    # launch_description.add_action(joint_state_publisher_node)
    launch_description.add_action(robot_state_publisher_node)
    # launch_description.add_action(cartographer_node)
    # launch_description.add_action(occupancy_grid_node)

    return launch_description