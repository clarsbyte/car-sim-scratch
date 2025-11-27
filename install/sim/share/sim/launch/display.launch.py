import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    rviz_args = []
    if os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read(), 'use_sim_time': True}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args
        )
    ])