#   SLAM:
#     $ ros2 launch sucky_rtabmap real_rtab_map.launch.py rviz:=true rtabmap_viz:=true
#
#   Rosbag:
#     $ ros2 bag play demo_mapping.db3 --clock

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    remappings=[
         ('rgb/image',       '/camera/d455/color/image_raw'),
         ('depth/image',     '/camera/d455/depth/image_rect_raw'),
         ('rgb/camera_info', '/camera/d455/color/camera_info'),
         ('scan',            '/scan')]

    parameters={
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001, # set to low to trust odometry from wheel encoders
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'approx_sync':True,
          'sync_queue_size': 30,
          'RGBD/NeighborLinkRefining': 'true',
          'RGBD/ProximityBySpace':     'true',
          'RGBD/ProximityByTime':      'false',
          'RGBD/ProximityPathMaxNeighbors': '10',
          'Reg/Strategy':              '1',
          'Vis/MinInliers':            '12',
          'RGBD/OptimizeFromGraphEnd': 'false',
          'RGBD/OptimizeMaxError':     '4',
          'Reg/Force3DoF':             'true',
          'Mem/STMSize':               '30',
          'RGBD/LocalRadius':          '5',
          'Icp/CorrespondenceRatio':   '0.2',
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.05',

          # 2D Grid Map 
          'Grid/Sensor': '2',                        # Use both scan and depth/point cloud
          'Grid/NormalsSegmentation': 'true',        # Enable surface normal filtering
          'Grid/FlatObstacleDetected': 'true',       # Keep tables, benches, etc.
          'Grid/MaxGroundHeight': '0.1',             # Anything below 15cm = ground
          'Grid/MaxObstacleHeight': '2.00',           # Filter out ceilings and tall objects
          'Grid/RayTracing': 'false',                 # Simulate free space
          'Grid/RangeMin': '0.3',                    # Ignore very close points
          'Grid/RangeMax': '5.0',                    # Sensor max range
          'Grid/MapFrameProjection': 'true',         # Project obstacles in map frame
          'Grid/NormalK': '30',                      # More stable normals
    }
    
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'compressed',
               'depth_image_transport':'compressedDepth',
               'approx_sync_max_interval': 0.02}],
            remappings=remappings),
        
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=['-d', os.path.join(
                get_package_share_directory('sucky_rtabmap'),
                'rviz',
                'real.rviz'
            )],
        ),
    ])
