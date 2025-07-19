from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os

def generate_launch_description():
    # Constants for paths to different files and folders
    gazebo_pkg_name = 'nav_rover_control'
    robot_name_in_model = 'ares8_rover'
    urdf_file_path = 'urdf/ares8_rover.urdf'
    world_file_path = 'worlds/sonoma_raceway.world'
    pkg_four_ws_control = get_package_share_directory('four_ws_control')
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    gazebo_pkg_share = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    world_path = os.path.join(gazebo_pkg_share, world_file_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world_path}.items(),

    )

    urdf_model_path = os.path.join(gazebo_pkg_share, urdf_file_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name_in_model,
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val,
                                   ],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_tire_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active', 'tire_controller'],
        output='screen'
    )

    load_str_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active', 'str_controller'],
        output='screen'
    )


    nodes = [
        # Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    load_tire_controller,
                    load_str_controller,
                ],
            )
        ),
        gazebo,
        spawn_entity,
    ]

    return LaunchDescription(nodes)
