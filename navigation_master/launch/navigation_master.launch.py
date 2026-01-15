from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

    Node(
        package='navigation_master',
        executable='navigation_master',
        name='navigation_master',
        parameters=[{'active_algorithm': 'potential_field', 
                     'publish_rate': 10.0}],
        output='screen'
    ),
    
    ])
