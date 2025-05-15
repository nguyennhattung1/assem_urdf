from launch import LaunchDescription  
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node  
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameters for all nodes
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
        
    # SLAM Toolbox params file
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffbot_base_controller'],
        output='screen'
    )
    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diffbot_base_controller/odom",
                "output_topic": "/odom",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

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
    )


    # Connection established
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/diffbot_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
    bridge_imu = Node (
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
        output='screen'
    )
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/collision_test/model/my_robot/link/base_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/collision_test/model/my_robot/link/base_link/sensor/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/collision_test/model/my_robot/link/base_link/sensor/depth_camera/depth_image', '/depth_image/image_raw'),
            ('/world/collision_test/model/my_robot/link/base_link/sensor/depth_camera/depth_image/points', 'depth/points')
        ],
        output='screen',
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                'rvizconfig',
                default_value=os.path.join(
                    get_package_share_directory('assem_urdf'),
                    'rviz',
                    'navigation_config.rviz'),
            ),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'slam_toolbox_params.yaml'),
                description='Full path to the SLAM Toolbox parameters file to use'
            ),
            declare_use_sim_time,
            bridge_clock,
            load_joint_state_controller,
            load_diff_drive_controller,
            # bridge_cmd_vel,
            bridge_lidar,
            bridge_camera,
            bridge_imu,
            relay_odom,
            relay_cmd_vel,
            slam_toolbox_node,
            rviz_node,
        ] )
    return ld
   
