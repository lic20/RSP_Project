from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch.actions
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions.launch_configuration import LaunchConfiguration

def generate_launch_description():

    robot_description_content = Command(
        [   PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("turtlebot_description"), "robots", "turtlebot_lidar.urdf.xacro"])
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/joint_states_k'), ('/tf', 'tf'),
            ('/tf_static', 'tf_static')]
    )

    kobuki_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('kobuki_node'),
                'launch', 'kobuki_node-launch.py')]),
        remappings=[
            ('/commands/velocity', '/cmd_vel')]
    )

    laser_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar.launch.py')])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file]
    )

    node_nav =  launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')]))
    
    node_slam = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')]))

    nodes_to_start = [
        robot_state_publisher_node,
        kobuki_node,
        laser_node,
        rviz_node,
        node_nav,
        node_slam
    ]


    return LaunchDescription(nodes_to_start)

