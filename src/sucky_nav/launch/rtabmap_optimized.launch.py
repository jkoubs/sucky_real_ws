#!/usr/bin/env python3
"""
RTAB-Map Localization-only (map->odom) using wheel odometry (odom->base_*).

Usage (real robot):
  ros2 launch sucky_rtabmap rtabmap_localization.launch.py \
    database_path:=/home/you/maps/plant_mapping.db \
    use_sim_time:=false rtabmap_viz:=true

Usage (with rosbag + /clock):
  ros2 launch sucky_rtabmap rtabmap_localization.launch.py \
    database_path:=/home/you/maps/plant_mapping.db \
    use_sim_time:=true rtabmap_viz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ---- Launch args ----
    use_sim_time   = LaunchConfiguration('use_sim_time')
    rtabmap_viz_on = LaunchConfiguration('rtabmap_viz')
    db_path        = LaunchConfiguration('database_path')

    # Camera + lidar topic remaps (adjust if your topics differ)
    remappings = [
        ('rgb/image',       '/camera/d455/color/image_raw'),
        ('depth/image',     '/camera/d455/depth/image_rect_raw'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan',            '/scan'),
        ('odom',            '/diffbot_base_controller/odom'),          # wheel odom (nav_msgs/Odometry)
    ]

    # ---- RTAB-Map parameters (types must be native, not strings) ----
    params = {
        # Frames (use base_link if that’s your TF root; otherwise base_footprint)
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',

        # We trust wheel odom; RTAB-Map publishes map->odom
        'odom_tf_linear_variance':  0.001,
        'odom_tf_angular_variance': 0.001,

        # Subscriptions
        'subscribe_rgbd': True,
        'subscribe_scan': True,     # use both camera and lidar (helps robustness)
        'approx_sync': True,
        'sync_queue_size': 30,

        # 2D (planar) localization
        'Reg/Force3DoF': True,

        # Registration strategy: 2 = Visual + ICP (good for RGB-D + lidar)
        'Reg/Strategy': 2,

        # Robustness knobs
        'RGBD/NeighborLinkRefining': True,
        'RGBD/ProximityBySpace': True,
        'RGBD/ProximityByTime': False,
        'RGBD/ProximityPathMaxNeighbors': 10,
        'Vis/MinInliers': 12,
        'RGBD/OptimizeFromGraphEnd': False,
        'RGBD/OptimizeMaxError': 4.0,

        # ICP (used by Strategy 2 or 1)
        'Icp/CorrespondenceRatio': 0.2,
        'Icp/PM': False,
        'Icp/PointToPlane': False,
        'Icp/MaxCorrespondenceDistance': 0.15,
        'Icp/VoxelSize': 0.05,

        # Keep memory lean
        'Mem/STMSize': 30,

        # Grid projection settings (kept for consistency with your 2D export)
        'Grid/Sensor': 2,                 # 0=laser,1=depth,2=both
        'Grid/NormalsSegmentation': True,
        'Grid/FlatObstacleDetected': True,
        'Grid/MaxGroundHeight': 0.10,
        'Grid/MaxObstacleHeight': 1.30,   # <-- your post-processed value
        'Grid/RayTracing': False,
        'Grid/RangeMin': 0.3,
        'Grid/RangeMax': 5.0,
        'Grid/NormalK': 30,
    }

    # Optional RGB-D sync helper (transport + small slack)
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[params, {
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'approx_sync_max_interval': 0.03
        }],
        remappings=remappings
    )

    # RTAB-Map in localization-only mode (no new nodes, use existing DB)
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[params, {
            'database_path': db_path,            # REQUIRED: your built map DB
            'Mem/IncrementalMemory': False,      # do not add new nodes
            'Mem/InitWMWithAllNodes': True,      # preload map graph
            'Rtabmap/StartNewMap': False         # never start a new map
        }],
        remappings=remappings
    )

    # Optional RTAB-Map visualization GUI
    rtabmap_viz = Node(
        condition=IfCondition(rtabmap_viz_on),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[params, {'database_path': db_path}],
        remappings=remappings
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path',
                              default_value='/home/you/maps/plant_mapping.db',
                              description='Path to EXISTING RTAB-Map .db to localize against'),
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use /clock (true for rosbag, false on robot)'),
        DeclareLaunchArgument('rtabmap_viz',
                              default_value='true',
                              description='Show RTAB-Map visualization UI'),
        SetParameter(name='use_sim_time', value=use_sim_time),
        rgbd_sync,
        rtabmap,
        rtabmap_viz,
    ])
