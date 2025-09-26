from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            remappings=[
                ("depth", "/camera/d455/depth/image_rect_raw"),
                ("depth_camera_info", "/camera/d455/depth/camera_info"),
                ("scan", "/scan_depth" ),
            ],
            parameters=[{
                "output_frame": "d455_link",
                "range_min": 0.6,
                "range_max": 3.0,
                "scan_time": 0.033,        # ~30 Hz depth stream
                "scan_height": 40,          # just the center plane
            }]
        )
    ])
