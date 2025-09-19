from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Launching turtlesim_node..."),
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),

        LogInfo(msg="Launching pose_logger node..."),
        Node(package='turtlesim_demo', executable='pose_logger', name='pose_logger'),

        LogInfo(msg="Launching circle_driver node..."),
        Node(package='turtlesim_demo', executable='circle_driver', name='circle_driver'),

        LogInfo(msg="Launching fibonacci_action_server node..."),
        Node(package='turtlesim_demo', executable='action_server', name='fibonacci_action_server'),

        LogInfo(msg="Launching fibonacci_action_client node..."),
        Node(package='turtlesim_demo', executable='action_client', name='fibonacci_action_client'),
    ])
