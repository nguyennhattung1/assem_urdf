# Ví dụ: src/assem_urdf/launch/ps4_teleop.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_config_filepath = os.path.join(
        get_package_share_directory('assem_urdf'), # Thay 'assem_urdf' nếu tên package của bạn khác
        'config', 
        'ps4_joy_config.yaml' # File cấu hình mapping cho teleop_twist_joy
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Thiết bị tay cầm, có thể khác
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_filepath]
        # No remapping needed since relay_cmd_vel in control_and_topic_init.launch.py
        # already relays from /cmd_vel to /diffbot_base_controller/cmd_vel_unstamped
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
        # Thêm các node khác của robot nếu cần (robot_state_publisher, controller_manager, etc.)
        # nếu bạn tạo launch file riêng chỉ để teleop.
        # Nếu thêm vào launch file hiện có, chỉ cần thêm 2 node trên.
    ])