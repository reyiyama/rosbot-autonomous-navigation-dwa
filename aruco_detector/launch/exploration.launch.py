from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frequency', default_value='1.0'),
        
        Node(
            package='aruco_detector',
            executable='aruco_marker_action',
            name='publish_marker',
            output='screen',
            parameters=[
                {'frequency': LaunchConfiguration('frequency')},
            ]
        ),
        Node(
            package='aruco_detector',
            executable='dwa_navigator',
            name='publish_navpath',
            output='screen',
            parameters=[
                {'frequency': LaunchConfiguration('frequency')},
            ]
        ),
    ])
