import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='robot')
    launch_description = LaunchDescription()
    package_name = 'robot_description'
    package_share = FindPackageShare(package = package_name).find(package_name)

    xacro_file = os.path.join(package_share, 'urdf/robot.urdf.xacro')
    urdf_file = os.path.join(package_share, 'urdf/robot.urdf')
    xacro_command = ExecuteProcess(
        cmd = [
            'xacro',xacro_file,
            PythonExpression(['str(','"robot_name:=',robot_name,'")']),
            '-o',urdf_file,
        ]
    )

    python_script = os.path.join(package_share, 'cartographer_config/cartographer_config.py')
    cartographer_src = os.path.join(package_share, 'cartographer_config/robot_config.lua')
    cartographer_dst = os.path.join(package_share, 'cartographer_config/use_robot_config.lua')
    config_command = ExecuteProcess(
        cmd = [
            'python3', python_script, robot_name,
            cartographer_src, cartographer_dst
        ]
    )

    launch_description.add_action(xacro_command)
    launch_description.add_action(config_command)

    return launch_description