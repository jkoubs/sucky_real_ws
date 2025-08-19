#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    # --- Args ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    db_path      = LaunchConfiguration('database_path')
    map_yaml     = LaunchConfiguration('map_yaml')
    rtabmap_viz  = LaunchConfiguration('rtabmap_viz')
    start_init_pose = LaunchConfiguration('start_initial_pose_node')

    # Default paths (adjust package/name to yours)
    nav_pkg = 'sucky_nav'
    default_map_yaml = join(get_package_share_directory(nav_pkg), 'maps', 'quad-shop-walkie-talkie-cleaned.yaml')
    default_db       = '/home/you/maps/plant_mapping.db'  # <-- your existing RTAB-Map DB

    # Topics/frames — tweak if yours differ
    remaps = [
        ('rgb/image',       '/camera/d455/color/image_raw'),
        ('depth/image',     '/camera/d455/depth/image_rect_raw'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan',            '/scan'),
        ('odom',            '/odom'),  # wheel odom
    ]

    # Shared RTAB-Map params (booleans/ints/floats only)
    rtab_params = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'odom_tf_linear_variance':  0.001,
        'odom_tf_angular_variance': 0.001,

        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'sync_queue_size': 30,

        'Reg/Force3DoF': True,
        'Reg/Strategy': 2,  # Visual+ICP
        'Vis/MinInliers': 12,
        'RGBD/NeighborLinkRefining': True,
        'RGBD/ProximityBySpace': True,
        'RGBD/ProximityByTime': False,
        'RGBD/ProximityPathMaxNeighbors': 10,
        'RGBD/OptimizeFromGraphEnd': False,
        'RGBD/OptimizeMaxError': 4.0,

        'Icp/CorrespondenceRatio': 0.2,
        'Icp/PM': False,
        'Icp/PointToPlane': False,
        'Icp/MaxCorrespondenceDistance': 0.15,
        'Icp/VoxelSize': 0.05,

        'Mem/STMSize': 30,

        # Keep these aligned with your exported 2D map (you said 1.30 m)
        'Grid/Sensor': 2,
        'Grid/NormalsSegmentation': True,
        'Grid/FlatObstacleDetected': True,
        'Grid/MaxGroundHeight': 0.10,
        'Grid/MaxObstacleHeight': 1.30,
        'Grid/RayTracing': False,
        'Grid/RangeMin': 0.3,
        'Grid/RangeMax': 5.0,
        'Grid/NormalK': 30,
    }

    # Nodes
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[rtab_params, {
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'approx_sync_max_interval': 0.03
        }],
        remappings=remaps
    )

    rtabmap_localize = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtab_params, {
            'database_path': db_path,        # EXISTING DB (built during mapping)
            'Mem/IncrementalMemory': False,  # no new nodes
            'Mem/InitWMWithAllNodes': True,  # preload map graph
            'Rtabmap/StartNewMap': False     # never start a new map
        }],
        remappings=remaps
    )

    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtab_params, {'database_path': db_path}],
        remappings=remaps
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml
        }]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': ['map_server']  # only map_server is lifecycle-managed
        }]
    )

    # Optional: your initial pose publisher (or just use RViz's 2D Pose Estimate)
    initial_pose_node = Node(
        condition=IfCondition(start_init_pose),
        package='sucky_nav',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true for rosbag with --clock'),
        DeclareLaunchArgument('database_path', default_value=default_db,
                              description='Path to EXISTING RTAB-Map .db'),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml,
                              description='YAML of the exported 2D map (at 1.30 m)'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true',
                              description='Show RTAB-Map visualization UI'),
        DeclareLaunchArgument('start_initial_pose_node', default_value='false',
                              description='Run initial pose publisher script'),
        SetParameter(name='use_sim_time', value=use_sim_time),

        rgbd_sync,
        rtabmap_localize,
        map_server,
        lifecycle,
        rtabmap_viz_node,
        initial_pose_node,
    ])
