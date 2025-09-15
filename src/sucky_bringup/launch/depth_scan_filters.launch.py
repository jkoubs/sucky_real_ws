from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_pkg = "sucky_bringup"  # <- change to your package name
    laser_filters_params = os.path.join(get_package_share_directory(bringup_pkg), "config", "laser_filters.yaml")

    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            parameters=[laser_filters_params],
            remappings=[("scan", "/scan_depth"), ("scan_filtered", "/scan_depth_filtered")],
            output="screen",
        )
    ])
