import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    declared_arguments = []

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    workspace_src_dir = os.path.join(os.environ.get('ROS_WORKSPACE', '/home/user/ros2_ws'), 'src', 'rl_fra2mo_description', 'rviz_conf')
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)
    
    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_file_name = LaunchConfiguration('file')

    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]), value_type=str)}

    # Dichiarazione dell'argomento "file"
    declared_arguments.append(
        DeclareLaunchArgument(
            "file",
            default_value="explore.rviz",
            description="Name of the RViz config file to use (without full path).",
        )
    )

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([workspace_src_dir, rviz_file_name])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
