import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Process the xacro file to expand includes and macros
    robot_desc = xacro.process_file(urdf_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--param-file', controller_config]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--param-file', controller_config]
    )

    # Delay controller spawners to ensure controller_manager is ready
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[diff_drive_spawner]
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_diff_drive
    ])