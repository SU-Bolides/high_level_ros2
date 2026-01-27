import launch
from launch import LaunchDescription
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
            package='potential_field_nav',
            executable='unified_potential_field',
            name='potential_field_node',
            output='screen',
            parameters=[
                {'max_speed': 0.04},
                {'min_speed': 0.015},
                {'max_steering_angle_deg': 40.0},
                {'influence_distance': 4.0},
                {'k_repulsive': 0.4},
                {'k_attractive': 1.0},
                {'smoothing_alpha': 0.3},
                {'front_obstacle_threshold': 2.0},
                {'critical_obstacle_distance': 0.5},
                {'repulsive_deadzone': 100.0}
            ],
            respawn=True
        ),
    ])