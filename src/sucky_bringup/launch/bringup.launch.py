import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    bringup_pkg = get_package_share_directory('sucky_bringup')
    robot_controllers_path = os.path.join(get_package_share_directory('sucky_bringup'),'config','sucky_controllers_optimized.yaml')
    joy_params = os.path.join(get_package_share_directory('sucky_bringup'),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory('sucky_bringup'),'config','twist_mux.yaml')
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')
    laser_filters_params = os.path.join(get_package_share_directory('sucky_bringup'), "config", "laser_filters.yaml")
    #ekf_params_file = os.path.join(get_package_share_directory('sucky_nav'), 'config', 'ekf.yaml')

    # Load and Process Xacro
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'robot.urdf.xacro')
    with open(xacro_file) as f:
        doc = xacro.parse(f)

    # Process the parsed Xacro document to generate the URDF XML
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Robot State Publisher - reduce output
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',  # Reduce console output
        parameters=[robot_description]
    )

    joystick_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': False}],
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel','/cmd_vel_joy')],
    )

    # Joystick controller for cyclone and doors
    joystick_controller_node = Node(
        package='sucky_bringup',
        executable='sucky_joy.py',
        name='joystick_controller',
        output='log',
        parameters=[joy_params, {'use_sim_time': False}]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_path, {'use_sim_time': False}],
        output="both",
        #remappings=[('/diffbot_base_controller/odom', '/odom'),]
        
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )


    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='log',  # Keep log output to reduce CPU
        parameters=[{'use_sim_time': False}],
        arguments=[
            tim_launch_file_path,
            'frame_id:=sick_link',          # <-- messages will use this frame
            'tf_publish_rate:=0.0',  # aleady pub by RSP (URDF)
            'hostname:=192.168.0.1',
            'min_ang:=-1.18',  # -70 degrees in radians
            'max_ang:=1.18',   # 70 degrees in radians
        ]
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            # get_package_share_directory('sucky_bringup'), 'launch', 'camera.launch.py'
            get_package_share_directory('sucky_bringup'), 'launch', 'camera_localization_rtabmap.launch.py'
        )])
    )

    depth_to_laserscan = Node(
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
            "range_max": 6.0,
            "scan_time": 0.033,        # ~30 Hz depth stream
            "scan_height": 1,          # just the center plane
        }]
    )

    laser_filters = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        parameters=[laser_filters_params],
        remappings=[("scan", "/scan_depth"), ("scan_filtered", "/scan_depth_filtered")],
        output="screen",
        )

    arduino_controller_node = Node(
        package='sucky_bringup',
        executable='arduino_controller.py',
        name='arduino_controller',
        output='log',
        parameters=[{
            'serial_port': '/dev/arduino',
            'baud_rate': 115200,
            'timeout': 2.0,
            'use_sim_time': False
        }]
    )


    return LaunchDescription([
        robot_state_publisher,
        joystick_node,
        teleop_node,
        joystick_controller_node,
        ros2_control_node,
        twist_mux,
        robot_controller_spawner,
        joint_state_broadcaster,
        sick_node,
        camera_node,
        depth_to_laserscan,
        laser_filters,
        arduino_controller_node,
    ])


    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='log',  # Reduce console output
    #     parameters=[ekf_params_file, {'use_sim_time': False}],
    # )


    # battery_monitor_node = Node(
    #     package='sucky_bringup',
    #     executable='battery_monitor.py',
    #     name='battery_monitor',
    #     output='log',  # Reduce output
    #     parameters=[{
    #         'serial_port': '/dev/ttyACM2', 
    #         'address': 128,
    #         'publish_rate': 0.1, 
    #         'min_voltage': 22.0,
    #         'max_voltage': 29.4,
    #         'use_sim_time': False
    #     }]
    # )



    # # ---- Include AMCL (and scan_relay) launch AFTER LiDAR starts ----
    # amcl_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('sucky_nav'), 'launch', 'amcl.launch.py')
    #     ])
    # )

    # start_amcl_after_lidar = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=sick_node,
    #         on_start=[amcl_launch]
    #     )
    # )