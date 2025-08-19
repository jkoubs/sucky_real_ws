from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sucky_nav_dir = get_package_share_directory('sucky_nav')
    nav2_params = os.path.join(sucky_nav_dir, 'config', 'nav2_fcpp.yaml')

    bt_xml_path = os.path.join(
        sucky_nav_dir,
        'behavior_trees',
        'spiral.xml'
    )

    # Individual Nav2 nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[
        ('/cmd_vel', '/cmd_vel_nav'),
    ]
    #     remappings=[
    #     ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
    # ]

    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}]
    )

    bt_navigator = Node(
        package='backported_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params, 
            {'use_sim_time': False},
            {'default_nav_to_pose_bt_xml': bt_xml_path}        
        ]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),  # input from controller_server
            ('cmd_vel_smoothed', '/diffbot_base_controller/cmd_vel_unstamped')  # final output
        ],
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    return LaunchDescription([
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager
    ])
