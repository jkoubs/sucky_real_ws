from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_topic   = LaunchConfiguration('input_topic')
    output_topic  = LaunchConfiguration('output_topic')
    target_hz     = LaunchConfiguration('target_hz')
    decimate      = LaunchConfiguration('decimate')
    restamp       = LaunchConfiguration('restamp')
    qos_depth     = LaunchConfiguration('qos_depth')
    frame_override = LaunchConfiguration('frame_id_override')

    return LaunchDescription([
        # ---- args (override on the command line if you like) ----
        DeclareLaunchArgument('input_topic',   default_value='/scan'),
        DeclareLaunchArgument('output_topic',  default_value='/scan_viz'),
        DeclareLaunchArgument('target_hz',     default_value='8.0'),   # throttle to ~8–10 Hz for RViz
        DeclareLaunchArgument('decimate',      default_value='2'),     # every Nth beam (1 = no decimation)
        DeclareLaunchArgument('restamp',       default_value='true'),  # stamp with "now" to make TF easy
        DeclareLaunchArgument('qos_depth',     default_value='5'),     # small depth so RViz is lightweight
        DeclareLaunchArgument('frame_id_override', default_value=''),  # leave empty to keep source frame

        Node(
            package='sucky_bringup',
            executable='scan_viz_relay.py',   # from your setup.py entry_point
            name='scan_viz_relay',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'target_hz': target_hz,
                'decimate': decimate,
                'restamp': restamp,
                'qos_depth': qos_depth,
                'frame_id_override': frame_override,
            }],
        ),
    ])
