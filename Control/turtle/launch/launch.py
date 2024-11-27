import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle'
    urdf_name='URDF1.urdf.xacro'
    urdf=os.path.join(get_package_share_directory('turtle'),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'

    # Set paths to different files
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    resources=os.path.join(get_package_share_directory('turtle'))
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    doc = xacro.process_file(urdf, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # Launch RViz2
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_path]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'turtle',
                   '-allow_renaming', 'false'],
    )

    # Set environment variable for GZ_SIM_RESOURCE_PATH
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resources
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("turtle"),
            "config",
            "joint_controller.yaml",
        ]
    )

    Velcoity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_vel"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
     
    return LaunchDescription([
        set_gz_sim_resource_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        #launch_rviz_node,
        gz_sim,
        spawn_entity,
        Velcoity_spawner,
        joint_broad_spawner,
    ])
