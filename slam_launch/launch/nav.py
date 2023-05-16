from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, LaunchConfiguration
import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    dir = get_package_share_directory('slam_launch')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            dir, 'config', 'map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

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
    )

    laser_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar.launch.py')])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name="rviz2",
        arguments=["-d", rviz_config_file]
    )

    node_nav = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')]),
        launch_arguments={'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,}.items())


    nodes_to_start = [
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        robot_state_publisher_node,
        kobuki_node,
        laser_node,
        rviz_node,
        node_nav,
    ]


    return LaunchDescription(nodes_to_start)

