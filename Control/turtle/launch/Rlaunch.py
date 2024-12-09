import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, AppendEnvironmentVariable
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
    package_name = 'turtle_on_land_demo'
    urdf_name='URDF1.urdf.xacro'
    world_sdf_file = 'world.sdf'

    urdf=os.path.join(get_package_share_directory('turtle_on_land_demo'),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'
    path_to_world_sdf = os.path.join(
        get_package_share_directory(package_name),
        'world',
        world_sdf_file)


    gz_args = f"-r -v 4 {path_to_world_sdf}"

    # Set paths to different files
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    resources=os.path.join(get_package_share_directory('turtle_on_land_demo'))
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    doc = xacro.process_file(urdf, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
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
        launch_arguments={"gz_args": f"-r -v 4 {path_to_world_sdf}"}.items(),
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
    resource_path = get_package_share_directory('turtle_on_land_demo')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(resource_path)).parent.resolve()))

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
    

    # Add the ros_gz_bridge node to bridge the joint states and commands
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Bridge joint states from ROS 2 to Gazebo transport
            #'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Bridge velocity commands if controlling with velocity
            #'/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            
            #'/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',

            #'/camera1_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            #'/model/robot1/pose@geometry_msgs/msg/Odometry[gz.msgs.Pose',

            #'/joint_states@geometry_msgs/msg/Transform[gz.msgs.Pose',

            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            # {
            #     'config_file': os.path.join(
            #         get_package_share_directory('landTurtle2'), 'configs', 'landturtle_bridge.yaml'
            #     ),
            #     #'expand_gz_topic_names': True,
            #     'use_sim_time': True,
            # }
        ],
    )
     

    delayed_velocity= TimerAction(
        period=3.0,  # Delay in seconds
        actions=[Velcoity_spawner],
    )

    delayed_joint= TimerAction(
        period=2.5,  # Delay in seconds
        actions=[joint_broad_spawner],
    )

    return LaunchDescription([
        set_env_vars_resources2,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch_rviz_node,
        gz_sim,
        spawn_entity,
        delayed_velocity,
        delayed_joint,
        bridge,
        
    ])
