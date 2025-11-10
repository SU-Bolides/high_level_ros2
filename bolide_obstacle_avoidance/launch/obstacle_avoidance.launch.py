import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():
    return LaunchDescription([
        # Nœud Lidar
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

        # Nœud STM32 (capteurs)
        Node(
            package='bolide_stm32',
            executable='stm32_node',
            name='stm32_node',
            output='screen',
            respawn=True
        ),

        # Nœud contrôle vitesse
        Node(
            package='bolide_stm32',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            output='screen',
            parameters=[
                {'minimal_speed' : 8.2}
            ],
            respawn=True
        ),

        # Nœud contrôle direction
        Node(
            package='bolide_direction',
            executable='cmd_dir_node',
            name='cmd_dir_node',
            output='screen',
            respawn=True
        ),

        # Nœud détection d'obstacles d'urgence
        Node(
            package='check_obstacle',
            executable='obstacle_checker',
            name='obstacle_checker',
            output='screen',
            parameters=[
                {'obstacle_distance': 0.3},
                {'debug': True},
                {'neutral_duration': 2.0},
                {'reverse_duration': 1.0},
                {'reverse_speed': -0.06}
            ],
            respawn=True
        ),

        # Nœud évitement d'obstacles principal
        Node(
            package='bolide_obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[
                {'front_threshold': 0.5},    # Distance seuil obstacle devant (m)
                {'side_threshold': 0.3},     # Distance seuil côtés (m)
                {'turn_speed': 0.3},         # Vitesse de rotation normalisée
                {'forward_speed': 0.1},      # Vitesse en avant (m/s)
                {'debug': True}              # Logs de debug
            ],
            respawn=True
        ),
    ])