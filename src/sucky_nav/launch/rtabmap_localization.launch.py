# rtabmap_localization.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    nav_pkg  = "sucky_nav"        # YAML map + initial_pose script
    # map_file  = join(get_package_share_directory(nav_pkg), "maps",   "quad-shop-walkie-talkie-cleaned.yaml")
    # map_file = join(get_package_share_directory(nav_pkg), "maps", "quad-shop-140cm-cleaned.yaml")
    map_file = join(get_package_share_directory(nav_pkg), "maps", "zone-C-140cm-rotated-cleaned-framed-v3.yaml")
    


    # db_file  = join(get_package_share_directory(nav_pkg), "rtabmap_database", "zone-B-demo-200cm.db")
    db_file  = "/home/sweepynvidia/Miscellaneous/rtabmap_database/zone-C-200cm.db"

    # Camera topics (RealSense D455)
    RGB_TOPIC   = "/camera/d455/color/image_raw"
    DEPTH_TOPIC = "/camera/d455/depth/image_rect_raw"
    INFO_TOPIC  = "/camera/d455/color/camera_info"
    SCAN_TOPIC  = "/scan"

    # Ensure /scan is up first (your existing relay)
    scan_relay = Node(
        package="sucky_bringup",
        executable="scan_relay.py",
        name="scan_relay",
        output="screen",
        respawn=True,
    )

    # rgbd_sync: publishes /rgbd_image for RTAB-Map
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        respawn=True,
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.08,   # was 0.03; compressed needs more slack
            "topic_queue_size": 10,
            "sync_queue_size": 30,
            "qos": 1,                 # Default/RELIABLE
            "qos_camera_info": 1,
            "rgb/image_transport": "compressed",
            "depth/image_transport": "compressedDepth",
        }],
        remappings=[
            ("rgb/image",       RGB_TOPIC),
            ("depth/image",     DEPTH_TOPIC),
            ("rgb/camera_info", INFO_TOPIC),
        ],
    )

    # Map server owns /map (Nav2 / coverage planner consumes this)
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=True,
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": map_file
        }],
    )

    # Lifecycle only for map_server (RTAB-Map isn't lifecycle)
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 4.0,
            "node_names": ["map_server"],
        }],
    )

    # RTAB-Map core (localization-only)
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        respawn=True,
        parameters=[{
            # Frames / timing
            "frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "use_sim_time": False,

            # Database (no "-d")
            "database_path": db_file,
            "Mem/IncrementalMemory": "False",
            "Mem/InitWMWithAllNodes": "True",

            # Inputs
            "subscribe_rgbd": True,   # expects /rgbd_image from rgbd_sync
            "subscribe_scan": True,
            "approx_sync": True,
            "sync_queue_size": 30,

            # Keep it LIGHT on Orin Nano
            "Rtabmap/DetectionRate": "8.0",
            "Kp/MaxFeatures": "900",
            "Vis/MinInliers": "18",
            "Vis/FeatureType": "8",            # GFTT/ORB
            "Vis/CorType": "0",                # feature matching
            "Mem/UseOdomFeatures": "false",    # IMPORTANT

            # Registration: Vis+ICP (robust), modest ICP cost
            "Reg/Strategy": "2",               # 0=Vis, 1=ICP, 2=VisIcp
            "Icp/PointToPlane": "true",
            "Icp/VoxelSize": "0.07",
            "Icp/MaxCorrespondenceDistance": "0.18",
            "Icp/CorrespondenceRatio": "0.2",

            # 2D grid projection (debug only; map_server owns /map)
            "Grid/2D": "true",
            "Grid/Sensor": "0",                # pin to laser to silence warning
            "Grid/MinObstacleHeight": "0.05",
            "Grid/MaxObstacleHeight": "1.4",
            "Grid/CellSize": "0.05",
            "Grid/NormalsSegmentation": "true",
            "Grid/FlatObstacleDetected": "true",
            "Grid/MaxGroundHeight": "0.1",
            "Grid/RangeMin": "0.3",
            "Grid/RangeMax": "5.0",
            "Grid/MapFrameProjection": "true",
            "Grid/NormalK": "30",
        }],
        remappings=[
            ("scan", SCAN_TOPIC),
            # Avoid clashing with map_server on /map (OccupancyGrid)
            ("/map", "/rtabmap/map"),
        ],
    )

    # Delay initial pose a bit so WM/words are ready
    initial_pose_node = TimerAction(
        period=2.0,
        actions=[Node(
            package="sucky_nav",
            executable="initial_pose_publisher.py",
            name="initial_pose_publisher",
            output="screen",
            parameters=[{"use_sim_time": False}]
        )]
    )

    # Start rgbd_sync + rtabmap + lifecycle only after /scan relay is up
    start_after_relay = RegisterEventHandler(
        OnProcessStart(
            target_action=scan_relay,
            on_start=[rgbd_sync, rtabmap, lifecycle]
        )
    )

    return LaunchDescription([
        scan_relay,
        map_server,
        start_after_relay,
        initial_pose_node,
    ])
