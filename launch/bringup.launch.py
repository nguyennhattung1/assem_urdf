import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessIO, OnProcessStart
from launch.events.process import ProcessIO, ProcessStarted
from typing import List, Union, Sequence
from ament_index_python.packages import get_package_share_directory
import os

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


def generate_launch_description():
    pkg_assem_urdf = FindPackageShare('assem_urdf')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    
    # Use get_package_share_directory for paths that need to be resolved immediately
    assem_urdf_dir = get_package_share_directory('assem_urdf')

    # Declare common launch arguments
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
    declare_run_headless_arg = DeclareLaunchArgument(
        'run_headless',
        default_value='False',
        description='Run in headless mode (no GUI)'
    )
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_assem_urdf, 'world', 'empty.sdf']),
        description='SDF World File for Gazebo'
    )
    declare_spawn_x_arg = DeclareLaunchArgument("x", default_value="-2.0", description="Model Spawn X Axis Value")
    declare_spawn_y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Model Spawn Y Axis Value")
    declare_spawn_z_arg = DeclareLaunchArgument("z", default_value="0.2", description="Model Spawn Z Axis Value")
    declare_use_localization_arg = DeclareLaunchArgument('use_localization', default_value='true', description='Whether to use robot_localization')
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_assem_urdf, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file for navigation'
    )
    declare_rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=PathJoinSubstitution([pkg_assem_urdf, 'rviz', 'navigation_config.rviz']),
        description='Full path to the RViz config file to use'
    )
    
    # SLAM Toolbox params file - use string path for immediate access
    slam_params_file = os.path.join(assem_urdf_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Define status messages for progress tracking
    gazebo_ready_message = "Gazebo is ready"
    control_ready_message = "Loaded robot controller"
    joy_teleop_ready_message = "Joy teleop started"
    diff_drive_loaded_message = "Loaded controller 'diffbot_base_controller'"
    toolbox_ready_message = "optimize_trajectory"  # A message that appears when SLAM toolbox is ready
    navigation_ready_message = "Creating bond timer"

    # 1. Include Gazebo launch
    gazebo_launch_file = PathJoinSubstitution([
        pkg_assem_urdf, 'launch', 'gazebo.launch.py'
    ])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'use_localization': LaunchConfiguration('use_localization'),
        }.items()
    )

    # 2. Control and Topic Init
    control_topic_init_launch_file = PathJoinSubstitution([
        pkg_assem_urdf, 'launch', 'control_and_topic_init.launch.py'
    ])
    
    control_topic_init = ExecuteProcess(
        name="launch_control_init",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("assem_urdf"),
                "launch",
                "control_and_topic_init.launch.py",
            ]),
            "use_sim_time:=true",
        ],
        shell=False,
        output="screen",
    )
    
    # Start control after a delay
    delayed_control = TimerAction(
        period=5.0,
        actions=[control_topic_init]
    )

    # 3. Joy Teleop launch
    joy_teleop = ExecuteProcess(
        name="launch_joy_teleop",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("assem_urdf"),
                "launch",
                "joy_teleop.launch.py",
            ]),
        ],
        shell=False,
        output="screen",
    )
    
    # Start joy teleop after the control has started
    joy_teleop_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=control_topic_init,
            on_start=[
                LogInfo(msg="Control and topic init started. Starting joy teleop..."),
                TimerAction(period=5.0, actions=[joy_teleop])
            ],
        )
    )
    
    # 4. SLAM Toolbox
    slam_toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ]),
            "use_sim_time:=True",
            ["params_file:=", slam_params_file]
        ],
        shell=False,
        output="screen",
    )
    
    # Start SLAM after diff drive controller is loaded
    slam_toolbox_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=joy_teleop,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(msg="Diff drive controller loaded. Starting SLAM Toolbox..."),
                    TimerAction(period=3.0, actions=[slam_toolbox]),
                ],
            ),
        )
    )

    # 5. Navigation
    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ]),
            "use_sim_time:=True",
            ["params_file:=", LaunchConfiguration('params_file')]
        ],
        shell=False,
        output="screen",
    )
    
    # Wait for SLAM to be ready, then start navigation
    navigation_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=slam_toolbox,
            on_stdout=on_matching_output(
                toolbox_ready_message,
                [
                    LogInfo(msg="SLAM Toolbox loaded. Starting navigation..."),
                    TimerAction(period=5.0, actions=[navigation]),
                ],
            ),
        )
    )
    
    # 6. RViz (start after navigation is ready)
    rviz_node = Node(
        condition=IfCondition(NotSubstitution(LaunchConfiguration('run_headless'))),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    
    # Start RViz after navigation is ready
    rviz_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [
                    LogInfo(msg="Navigation ready. Starting RViz..."),
                    rviz_node,
                ],
            ),
        )
    )
    
    # Ready notification
    success_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[
                LogInfo(msg="Ready for navigation! All components started successfully."),
            ],
        )
    )

    return launch.LaunchDescription([
        declare_use_sim_time_arg,
        declare_headless_arg,
        declare_run_headless_arg,
        declare_world_arg,
        declare_spawn_x_arg,
        declare_spawn_y_arg,
        declare_spawn_z_arg,
        declare_use_localization_arg,
        declare_params_file_arg,
        declare_rvizconfig_arg,
        
        gazebo_launch,
        delayed_control,
        joy_teleop_handler,
        slam_toolbox_handler,
        navigation_handler,
        rviz_handler,
        success_handler,
    ]) 