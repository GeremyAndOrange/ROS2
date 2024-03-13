import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_persec = LaunchConfiguration('publish_persec', default='1.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_world_path = os.path.join(package_share, 'world/hard.world')
    start_gazebo_cmd = ExecuteProcess(
        cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
    )

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

    launch_description.add_action(start_gazebo_cmd)
    # launch_description.add_action(occupancy_grid_node)

    return launch_description