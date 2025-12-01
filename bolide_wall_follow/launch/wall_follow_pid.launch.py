import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyLIDAR'},
                {'serial_baudrate': 256000},
                {'frame_id': 'laser_frame'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Express'}
            ],
            respawn=True
        ),
        Node(
            package='bolide_stm32',
            executable='stm32_node',
            name='stm32_node',
            output='screen',
            respawn=True
        ),
        Node(
            package='bolide_stm32',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            output='screen',
            respawn=True
        ),
        Node(
            package='bolide_direction',
            executable='cmd_dir_node',
            name='cmd_dir_node',
            output='screen',
            respawn=True
        ),
        Node(
            package='check_obstacle',
            executable='obstacle_checker',
            name='obstacle_checker',
            output='screen',
            respawn=True
        ),
        Node(
            package='bolide_wall_follow',
            executable='wall_follow_pid',
            name='wall_follow_pid',
            output='screen',
            parameters=[
                {'kp': 12.0},
                {'ki': 0.0},
                {'kd': 1.0},
                {'target_distance': 0.3},
                {'wall_side': -90.0},
                {'front_angle_range': 15.0},
                {'obstacle_threshold': 1.0},
                {'max_speed': 0.04},
                {'min_speed': 0.02}
            ],
            respawn=True
        ),
    ])
