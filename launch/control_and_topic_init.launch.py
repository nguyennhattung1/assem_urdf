from launch import LaunchDescription  
from launch.actions import ExecuteProcess
from launch_ros.actions import Node  

def generate_launch_description():
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

    # bridge_camera = Node(
    #     package='ros_ign_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/world/robocon2025_map/model/my_robot/link/ban_bong/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
    #         '/world/robocon2025_map/model/my_robot/link/ban_bong/sensor/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #     ],
    #     remappings=[
    #         ('/world/robocon2025_map/model/my_robot/link/ban_bong/sensor/depth_camera/depth_image', '/depth_image/image_raw'),
    #         ('/world/robocon2025_map/model/my_robot/link/ban_bong/sensor/depth_camera/depth_image/points', 'depth/points')
    #     ],
    #     output='screen',
    # )
    rviz = Node(    
        package='rviz2',
        executable='rviz2',
    )

    ld = LaunchDescription(
        [
            load_joint_state_controller,
            load_diff_drive_controller,
            bridge_clock,
            # bridge_cmd_vel,
            # bridge_camera,
            rviz,
            # bridge_cmd_slide_mid,
            # bridge_cmd_slide_up,
        ] )
    return ld
   
