from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    nav_pkg = "sucky_nav"  
    map_file  = join(get_package_share_directory(nav_pkg), "maps",   "quad-shop-walkie-talkie-cleaned.yaml")
    amcl_config = join(get_package_share_directory(nav_pkg), "config", "amcl.yaml")

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

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=True,
        parameters=[
            {"use_sim_time": False},
            amcl_config,
            {"scan_topic": "scan"} 
        ],
        # Or remap instead:
        # remappings=[("scan", "/scan/filtered")],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 4.0,
            "node_names": ["map_server", "amcl"],
        }],
    )

    initial_pose_node = Node(
        package='sucky_nav',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen'
    )

    return LaunchDescription([map_server, amcl, lifecycle, initial_pose_node])