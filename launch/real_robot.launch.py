import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Package configuration
    package_description = "assem_urdf"
    package_directory = get_package_share_directory(package_description)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Set to false for real hardware
        description='Whether to use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF Configuration with use_sim=false for real hardware
    urdf_file = 'assem_urdf.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    print(f"Loading URDF/XACRO from: {robot_desc_path}")
    robot_description_content = Command(['xacro ', robot_desc_path, ' use_sim:=false'])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Controller Manager Node with hardware params 
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            os.path.join(package_directory, "config", "hardware_params.yaml"),
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Joint State Broadcaster - required for tf
    # Only spawn after controller_manager is started
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "info"],
        output="screen",
    )

    # Diff Drive Controller
    # Only spawn after controller_manager is started
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "info"],
        output="screen",
    )

    # Create event handlers to spawn controllers only after controller_manager has started
    joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    diff_drive_controller_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[diff_drive_controller_spawner]
        )
    )

    # EKF params file
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'ekf.yaml'),
        description='Full path to the EKF parameters file'
    )

    # Node EKF for sensor fusion (odometry and IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'), '--ros-args', '--log-level', 'info']
    )
    
    declare_rvizconfig = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_directory('assem_urdf'),
            'rviz',
            'navigation_config.rviz'),
    )
    
    # Launch joystick teleop
    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_directory, 'launch', 'joy_teleop.launch.py')
        )
    )
    
    # Command relayer for cmd_vel
    # Only create relay after the diff drive controller is spawned
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diffbot_base_controller/cmd_vel_unstamped",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Event to start relay only after diff drive controller is loaded
    relay_cmd_vel_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_controller_spawner,
            on_start=[relay_cmd_vel]
        )
    )

    # SLAM Toolbox params file
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM Toolbox parameters file to use'
    )
    
    # SLAM Toolbox for mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Combine all launch components

    robot_state_publisher_node,
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_ekf_params_file,
        declare_rvizconfig,
        declare_slam_params_file,
        robot_state_publisher_node,
        controller_manager_node,
        # Replaced direct spawner nodes with event handlers
        joint_state_broadcaster_event,
        diff_drive_controller_event,
        relay_cmd_vel_event,  # Adding relay event handler instead of direct node
        ekf_node,
        slam_toolbox_node,
        joy_teleop_launch,
        rviz_node,
    ])

    return ld 