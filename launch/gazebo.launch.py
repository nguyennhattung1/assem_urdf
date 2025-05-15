import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Package configuration
    package_description = "assem_urdf"
    package_directory = get_package_share_directory(package_description)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use sim time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF Configuration
    urdf_file = 'assem_urdf.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    print(f"Loading URDF/XACRO from: {robot_desc_path}")
    robot_description_content = Command(['xacro ', robot_desc_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # Spawn Position Arguments
    declare_spawn_x = DeclareLaunchArgument(
        "x", 
        default_value="-2.0",
        description="Model Spawn X Axis Value"
    )
    declare_spawn_y = DeclareLaunchArgument(
        "y", 
        default_value="0.0",
        description="Model Spawn Y Axis Value"
    )
    declare_spawn_z = DeclareLaunchArgument(
        "z", 
        default_value="0.5",
        description="Model Spawn Z Axis Value"
    )

    # Robot Spawn Node
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # Gazebo Resource Path Configuration
    # install_dir_path = (get_package_prefix(package_description) + "/share")
    # robot_meshes_path = os.path.join(package_directory, "meshes")
    # gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    pkg_share = get_package_share_directory(package_description)
    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models"),
        os.path.join(pkg_share, "world"),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ])

    # Environment variables for Gazebo
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_RESOURCE_PATH': gz_models_path,
        'IGN_GAZEBO_RENDER_ENGINE': 'ogre2',
        'IGN_GAZEBO_RENDER_ENGINE_PATH': os.environ.get('IGN_GAZEBO_RENDER_ENGINE_PATH', '')
    }

    # World Configuration
    world_file = "empty.sdf"
    world_file_path = os.path.join(package_directory, "world", world_file)

    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

    print(f"Loading world file from: {world_file_path}")

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_file_path,
        description="SDF World File"
    )

    # Khai báo headless argument trước khi sử dụng
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo in headless mode"
    )

    run_headless = LaunchConfiguration("headless")

    # Set verbosity level for Gazebo
    # Higher values (0-4) provide more detailed output for debugging
    # 0: Error, 1: Warning, 2: Info, 3: Debug, 4: Verbose
    gz_verbosity = "4"  

    # Gazebo launch configuration
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, "-s", "--headless-rendering", world_file_path],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )

    gazebo = ExecuteProcess(
        condition=UnlessCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, world_file_path],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )

    # EKF Node
    declare_use_localization = DeclareLaunchArgument(
        'use_localization',
        default_value='False',
        description='Whether to use robot_localization'
    )

    use_localization = LaunchConfiguration('use_localization')
    

    robot_localization_node = Node(
        condition=IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(package_directory, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Combine all launch components
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_headless_arg,
        declare_use_localization,
        robot_state_publisher_node,
        # Spawn position parameters
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gz_spawn_entity,
        declare_world_arg,
        # gz_sim,
        gazebo,
        gazebo_headless,
        # robot_localization_node,
    ])

    return ld
   
