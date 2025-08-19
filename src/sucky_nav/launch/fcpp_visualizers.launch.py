from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sucky_nav',
            executable='path_visualizer.py',
            name='path_visualizer',
            output='screen'
        ),
        Node(
            package='sucky_nav',
            executable='cleaning_visualizer.py',
            name='cleaning_visualizer',
            output='screen'
        )
    ])
