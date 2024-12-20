import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

def generate_launch_description():

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)

    models_path = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'models')
    world_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), "worlds", "leonardo_race_field.sdf")

    aruco_ros_file = os.path.join(
        FindPackageShare('aruco_ros').find('aruco_ros'),
        'launch',
        'single.launch.py'
        )
    
    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time', default_value='true', description='Use simulation/Gazebo clock')

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )
    
    # # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{"use_sim_time": True}]
    )

    declared_arguments = []
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument('gz_args', default_value=world_file, description='Path to the Gazebo world file'),
        DeclareLaunchArgument('marker_id', default_value='582', description='Aruco Marker ID'),
        DeclareLaunchArgument('marker_size', default_value='0.34', description='Size of the Aruco marker in meters'),
        DeclareLaunchArgument('eye', default_value='left', choices=['left', 'right'], description='Eye (left or right)'),
        DeclareLaunchArgument('marker_frame', default_value='aruco_marker_frame', description='Frame for the Aruco marker'),
        DeclareLaunchArgument('reference_frame', default_value='', description='Reference frame for the Aruco marker'),
        DeclareLaunchArgument(
            'corner_refinement', 
            default_value='LINES', 
            choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'], 
            description='Corner refinement method for Aruco marker detection'
        ),
    ]
    
    
    
    # Gazebo simulation launch description
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # Include the Aruco ROS launch description
    aruco_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_ros_file),
        launch_arguments={
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'eye': LaunchConfiguration('eye'),
            'marker_frame': LaunchConfiguration('marker_frame'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
        }.items()
    )

    position = [-3.0, 3.5, 0.1]
    RPY = [0.0, 0.0, -1.57]

    # Define a Node to spawn the robot in the Gazebo simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),
                    "-R", str(RPY[0]),
                    "-P", str(RPY[1]),
                    "-Y", str(RPY[2])]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                   #'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'], 
        output='screen'
    )


    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen',
    )


    odom_tf = Node(
        package='rl_fra2mo_description',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    laser_id_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='lidar_staticTF',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'laser_frame', 'fra2mo/base_footprint/laser_frame'],
                     parameters=[{"use_sim_time": True}]
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('rl_fra2mo_description'), "config/ekf.yaml")]
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace="fra2mo"
    )
 
    ign = [gazebo_ignition, gz_spawn_entity]
    nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, *ign, aruco_ros, bridge, bridge_camera,
                      odom_tf, laser_id_link_tf, ign_clock_bridge]

    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + declared_arguments + nodes_to_start)