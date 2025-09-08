from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sucky_nav_dir = get_package_share_directory('sucky_nav')
    nav2_params = os.path.join(sucky_nav_dir, 'config', 'nav2_opennav_coverage.yaml')

    coverage_server = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='coverage_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
        # arguments=['--ros-args', '--log-level', 'debug']
    )


    lifecycle_manager_coverage = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['coverage_server']
        }]
    )

    return LaunchDescription([
        coverage_server,
        lifecycle_manager_coverage
    ])