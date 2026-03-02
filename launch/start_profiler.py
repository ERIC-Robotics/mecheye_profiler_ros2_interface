from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('enable_profiler_1', default_value='true',
                              description='Launch profiler 1'),
        DeclareLaunchArgument('enable_profiler_2', default_value='true',
                              description='Launch profiler 2'),
        DeclareLaunchArgument('enable_profiler_3', default_value='true',
                              description='Launch profiler 3'),
        DeclareLaunchArgument('enable_profiler_4', default_value='true',
                              description='Launch profiler 4'),

        DeclareLaunchArgument('ip_1', default_value='192.168.0.37',
                              description='IP address of profiler 1'),
        DeclareLaunchArgument('ip_2', default_value='192.168.0.38',
                              description='IP address of profiler 2'),
        DeclareLaunchArgument('ip_3', default_value='192.168.0.39',
                              description='IP address of profiler 3'),
        DeclareLaunchArgument('ip_4', default_value='192.168.0.40',
                              description='IP address of profiler 4'),
    ]

    profiler_1 = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler/point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_1'))
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler/textured_point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_1'))
        ),
        Node(
            package='mecheye_profiler_ros_interface', executable='start',
            name='mechmind_profiler_publisher_service', output='screen',
            parameters=[{'save_file': False}, {'profiler_ip': LaunchConfiguration('ip_1')}],
            condition=IfCondition(LaunchConfiguration('enable_profiler_1'))
        ),
    ]

    profiler_2 = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_2/point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_2'))
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_2/textured_point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_2'))
        ),
        Node(
            package='mecheye_profiler_ros_interface', executable='start2',
            name='mechmind_profiler_publisher_service_2', output='screen',
            parameters=[{'save_file': False}, {'profiler_ip': LaunchConfiguration('ip_2')}],
            condition=IfCondition(LaunchConfiguration('enable_profiler_2'))
        ),
    ]

    profiler_3 = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_3/point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_3'))
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_3/textured_point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_3'))
        ),
        Node(
            package='mecheye_profiler_ros_interface', executable='start3',
            name='mechmind_profiler_publisher_service_3', output='screen',
            parameters=[{'save_file': False}, {'profiler_ip': LaunchConfiguration('ip_3')}],
            condition=IfCondition(LaunchConfiguration('enable_profiler_3'))
        ),
    ]

    profiler_4 = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_4/point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_4'))
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', '/mechmind_profiler_4/textured_point_cloud'],
            condition=IfCondition(LaunchConfiguration('enable_profiler_4'))
        ),
        Node(
            package='mecheye_profiler_ros_interface', executable='start4',
            name='mechmind_profiler_publisher_service_4', output='screen',
            parameters=[{'save_file': False}, {'profiler_ip': LaunchConfiguration('ip_4')}],
            condition=IfCondition(LaunchConfiguration('enable_profiler_4'))
        ),
    ]

    return LaunchDescription(args + profiler_1 + profiler_2 + profiler_3 + profiler_4)
