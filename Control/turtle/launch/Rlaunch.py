import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import TimerAction, IncludeLaunchDescription, AppendEnvironmentVariable, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle'
    urdf_name='URDF1.urdf.xacro'
    world_sdf_file = 'fws_robot_world.sdf'
     
   

    #Set paths to different files and packages
    urdf=os.path.join(get_package_share_directory(package_name),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'
    path_to_world_sdf = os.path.join( get_package_share_directory(package_name),'world', world_sdf_file)
    resource_path = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    #Gazebo models path
    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    #Rviz config file
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    #URDF Parser
    doc = xacro.process_file(urdf, mappings={'use_sim' : 'true'})

    #Robot description argument
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}

    #Set the use_sim_time parameter globally
    set_sim_time = SetLaunchConfiguration('use_sim_time', 'true')
   
    # Nodes
    #Robot_state_publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    #Joint_state_publisher Node
    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    #Rviz Node; Launches RVIZ
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_path],
        parameters=[
        {'tf_buffer_size': 120.0},  # Increase transform buffer size
     ]
    )

    # Gazebo Node: Launches Gazebo
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

    #Spawns entity Node
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
    

    #Gazebo resources
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(resource_path)).parent.resolve()))

    #Joint Vel Controller Node; Initiazlizes joint velocity control
    Velcoity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_vel"],
    )

    #Joint state broadcaster Node; Initializes Joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    

    # Bridge Node: Initializes lidar scan topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   ],
        output='screen',
    )
     
    #3 second delayed velocity spawner
    delayed_velocity= TimerAction(
        period=3.0,  # Delay in seconds
        actions=[Velcoity_spawner],
    )

    #2.5 second delayed joint state broadcaster spawner
    delayed_joint= TimerAction(
        period=2.5,  # Delay in seconds
        actions=[joint_broad_spawner],
    )
    

    velocity_joint_publisher= Node(
        package=package_name,
        executable='velocity_publisher.py',
        name='vel_publisher',
        output='screen',
    )

    h_slam_node= Node(
        package= package_name,
        executable='hslam.py',
        name='slam',
        output='screen',
    )

    odometry_publisher=Node(
        package=package_name,
        executable='odometry_publisher.py',
        name='odom_publish',
        output='screen',
    )

    delayed_publisher= TimerAction(
        period=4.0,  #4 Second Delay
        actions=[velocity_joint_publisher],
    )

    delayed_odom=TimerAction(
        period=5.0, #4 Second Delay
        actions=[odometry_publisher],
    )

    delayed_slam=TimerAction(
        period=5.0,
        actions=[h_slam_node],
    )

    

    return LaunchDescription([
        set_sim_time,
        set_env_vars_resources2,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch_rviz_node,
        gz_sim,
        spawn_entity,
        delayed_velocity,
        delayed_joint,
        bridge,
        delayed_publisher,
        #h_slam_node,
        #delayed_odom,
        
    ])
