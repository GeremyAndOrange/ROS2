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

    launch_description.add_action(xacro_command)

    return launch_description