import os  

from ament_index_python.packages import get_package_prefix, get_package_share_directory  
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter  
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_description = "assem_urdf" 
    package_directory = get_package_share_directory(package_description)

    urdf_file = 'assem_urdf.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    # Check if the file exists
    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    print(f"Loading URDF/XACRO from: {robot_desc_path}")

    robot_description_content = Command(['xacro ', robot_desc_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
    )

    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.2",
                                            description="Model Spawn Z Axis Value")

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


    # World Initialized


    # Get Package Description and Directory #

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    # Load Empty World SDF from Gazebo Sim Package #
    world_file = "robocon2025_map.sdf"
    world_file_path = os.path.join(package_directory, "world", world_file)
    world_config = LaunchConfiguration("world")

    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=["-v 4 ",world_file_path],
                                              description="SDF World File")
    
    # Declare GazeboSim Launch #
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": world_config}.items(),
    )

    ld = LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,

            declare_world_arg,
            gz_sim,

        ] )
    return ld
   
