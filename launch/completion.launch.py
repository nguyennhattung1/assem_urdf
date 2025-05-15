import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessIO
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from typing import List, Union, Sequence

def on_matching_output(matcher: str, result: List):
    """Create an event handler that triggers when output contains a specific string."""
    def on_output(event: ProcessIO) -> Union[List, None]:
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result
        return None
    return on_output

def generate_launch_description():
    pkg_assem_urdf = get_package_share_directory('assem_urdf')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Declare common launch arguments that might be needed by included files
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo in headless mode'
    )
    # Add other arguments from gazebo.launch.py if you want to control them from here
    # For example, world, spawn position (x, y, z), use_localization
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_assem_urdf, 'world', 'robotcon2025.sdf']), # Default world from gazebo.launch.py
        description='SDF World File for Gazebo'
    )
    declare_spawn_x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Model Spawn X Axis Value")
    declare_spawn_y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Model Spawn Y Axis Value")
    declare_spawn_z_arg = DeclareLaunchArgument("z", default_value="0.2", description="Model Spawn Z Axis Value")
    declare_use_localization_arg = DeclareLaunchArgument('use_localization', default_value='False', description='Whether to use robot_localization')
    
    # Custom SLAM Toolbox params file from assem_urdf package
    custom_slam_params_file = os.path.join(pkg_assem_urdf, 'config', 'slam_toolbox_params.yaml')

    # Search strings for event triggers - these should match actual output from processes
    diff_drive_loaded_message = "Diff drive controller loaded"
    joy_teleop_loaded_message = "Joy teleop loaded"
    slam_toolbox_loaded_message = "Slam toolbox loaded"

    # 1. Include Gazebo launch
    gazebo_launch = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("assem_urdf"),
                    "launch",
                    "gazebo.launch.py",
                ]
            ),
            "use_rviz:=false",
            ["use_sim_time:=", LaunchConfiguration('use_sim_time')],
            ["headless:=", LaunchConfiguration('headless')],
            ["world:=", LaunchConfiguration('world')],
            ["x:=", LaunchConfiguration('x')],
            ["y:=", LaunchConfiguration('y')],
            ["z:=", LaunchConfiguration('z')],
            ["use_localization:=", LaunchConfiguration('use_localization')],
        ],
        shell=False,
        output="screen",
    )
    
    # 2. Include Control and Topic Init launch after 15 seconds
    control_topic_init_launch = ExecuteProcess(
        name="launch_control_topic_init",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("assem_urdf"),
                    "launch",
                    "control_and_topic_init.launch.py",
                ]
            ),
        ],
        shell=False,
        output="screen",
    )

    # Use TimerAction instead of event handler to start after 15 seconds
    delayed_control_topic_init = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="Starting Control and Topic Init after 15-second delay..."),
            control_topic_init_launch
        ]
    )

    # 3. Include Joy Teleop launch
    joy_teleop_launch = ExecuteProcess(
        name="launch_joy_teleop",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("assem_urdf"),
                    "launch",
                    "joy_teleop.launch.py",
                ]
            ),
        ],
        shell=False,
        output="screen",
    )
    waiting_joy_teleop = RegisterEventHandler(
        OnProcessIO(
            target_action=control_topic_init_launch,
            on_stdout=on_matching_output(
                joy_teleop_loaded_message,
                [
                    LogInfo(
                        msg="Joy teleop loaded. Starting Joy Teleop..."
                    ),
                    joy_teleop_launch,
                ],
            )
        )
    )

    # 4. SLAM Toolbox with custom parameters (using base_link instead of base_footprint)
    slam_toolbox_launch = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["params_file:=", custom_slam_params_file]
        ],
        shell=False,
        output="screen",
    )
    waiting_slam_toolbox = RegisterEventHandler(
        OnProcessIO(
            target_action=joy_teleop_launch,
            on_stdout=on_matching_output(
                slam_toolbox_loaded_message,
                [
                    LogInfo(
                        msg="Slam toolbox loaded. Starting SLAM Toolbox..."
                    ),
                    slam_toolbox_launch,
                ],
            )
        )
    )
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_headless_arg,
        declare_world_arg,
        declare_spawn_x_arg,
        declare_spawn_y_arg,
        declare_spawn_z_arg,
        declare_use_localization_arg,
        gazebo_launch,
        delayed_control_topic_init,
        waiting_joy_teleop,
        waiting_slam_toolbox,
    ])
    
    
    
