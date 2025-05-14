import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_assem_urdf = get_package_share_directory('assem_urdf')

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
        default_value=PathJoinSubstitution([pkg_assem_urdf, 'world', 'bookstore.sdf']), # Default world from gazebo.launch.py
        description='SDF World File for Gazebo'
    )
    declare_spawn_x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Model Spawn X Axis Value")
    declare_spawn_y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Model Spawn Y Axis Value")
    declare_spawn_z_arg = DeclareLaunchArgument("z", default_value="0.2", description="Model Spawn Z Axis Value")
    declare_use_localization_arg = DeclareLaunchArgument('use_localization', default_value='true', description='Whether to use robot_localization')


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

    # 2. Include Control and Topic Init launch, delayed by 15 seconds
    control_topic_init_launch_file = PathJoinSubstitution([
        pkg_assem_urdf, 'launch', 'control_and_topic_init.launch.py'
    ])
    control_topic_init_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_topic_init_launch_file)
        # This launch file doesn't seem to have explicit launch arguments
        # that need to be passed from here based on its current content.
    )

    delayed_control_topic_init = TimerAction(
        period=15.0,
        actions=[control_topic_init_launch]
    )

    # 3. Include Joy Teleop launch
    joy_teleop_launch_file = PathJoinSubstitution([
        pkg_assem_urdf, 'launch', 'joy_teleop.launch.py'
    ])
    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_teleop_launch_file)
        # This launch file also doesn't seem to have explicit launch arguments
        # that need to be passed from here based on its current content.
    )

    delayed_joy_teleop = TimerAction(
        period=20.0,  # Delay by 20 seconds
        actions=[joy_teleop_launch]
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
        delayed_joy_teleop,
    ]) 