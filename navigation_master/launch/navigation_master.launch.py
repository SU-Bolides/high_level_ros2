import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch argument to choose which high-level algorithm to start
    declare_alg_arg = DeclareLaunchArgument(
        'active_algorithm',
        default_value='potential_field',
        description="Active algorithm: 'potential_field' or 'wall_follow_pid'"
    )

    active_algorithm = LaunchConfiguration('active_algorithm')

    # Include the potential_field launch if selected
    potential_field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('potential_field_nav'),
            'launch',
            'potential_field.launch.py'
        )),
        condition=IfCondition(PythonExpression(["'", active_algorithm, "' == 'potential_field'"]))
    )

    # Include the wall_follow_pid launch if selected
    wall_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bolide_wall_follow'),
            'launch',
            'wall_follow_pid.launch.py'
        )),
        condition=IfCondition(PythonExpression(["'", active_algorithm, "' == 'wall_follow_pid'"]))
    )

    # The navigation master node republishes the chosen algorithm commands on /cmd_vel and /cmd_dir
    navigation_master_node = Node(
        package='navigation_master',
        executable='navigation_master',
        name='navigation_master',
        parameters=[{'active_algorithm': active_algorithm, 'publish_rate': 10.0}],
        output='screen'
    )

    return LaunchDescription([
        declare_alg_arg,
        potential_field_launch,
        wall_follow_launch,
        navigation_master_node
    ])
