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
            PathJoinSubstitution([FindPackageShare("turtlebot_description"), "robots", "robot_all.urdf.xacro"])
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot_gazebo"), "launch", "view.rviz"]
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot_gazebo"), "config", "my_world.sdf"]
    )

    # controller_file = PathJoinSubstitution(
    #     [FindPackageShare("open_manipulator_description"), "config", "controller.yaml"]
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/world/zls/model/turtlebot_system/joint_state')]
    )

    node_bridge1 = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="bridge1",
        arguments=["/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"],
        remappings=[
            ('/lidar', '/scan')]
    )

    node_bridge2 = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="bridge2",
        arguments=["/camera@sensor_msgs/msg/Image[ignition.msgs.Image"]
    )

    node_bridge3 = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="bridge3",
        arguments=["/world/zls/model/turtlebot_system/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model"]
    )

    node_bridge4 = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="bridge4",
        arguments=["/model/turtlebot_system/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                   "/model/turtlebot_system/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                   "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
        remappings=[
            ('/model/turtlebot_system/odometry', '/odom'),
             ('/model/turtlebot_system/cmd_vel', '/cmd_vel')]
    )

    load_joint_state_broadcaster= launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller= launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'forward_position_controller'],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        name="spawn_model",
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-world', 'zls',
                   '-z','0.1',
                   '-x', '0.1',
                   '-y', '0.1'],
    )

    node_gazebo =  launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('ign_args', ["-r -v 4 ", world_file])])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file]
    )

    nodes_to_start = [
        node_gazebo,
        robot_state_publisher_node,
        gz_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_bridge1,
        node_bridge2,
        node_bridge3,
        node_bridge4,
        rviz_node
    ]


    return LaunchDescription(nodes_to_start)

