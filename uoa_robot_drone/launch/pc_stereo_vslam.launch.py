"""
stereo_indoor_demo.launch.py

Launch demo for indoor mapping with a ZED2 stereo/RGBD camera and RTAB-Map.

Pre-requisites:
  1. Source your ROS2 workspace: `source ~/colcon_ws/install/setup.bash`
  2. Bring up the ZED2 driver: the launch below will include it automatically.
  3. Ensure your network is configured with machine name(PC: ros2-pc, Orin: nano, RPi: raspberrypi).

Example:
  $ ros2 launch rtabmap_demos stereo_indoor_demo.launch.py

Launch arguments:
  use_sim_time  – if true, use /clock
  localization  – if true, run in localization-only mode
  rtabmap_viz   – if true, start the RTAB-Map GUI
  rviz          – if true, start RViz2
  rviz_cfg      – path to RViz config
  camera_model  – ZED model to pass to zed_wrapper (default: zed2)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time   = LaunchConfiguration('use_sim_time')
    localization   = LaunchConfiguration('localization')
    rtabmap_viz    = LaunchConfiguration('rtabmap_viz')
    rviz           = LaunchConfiguration('rviz')
    rviz_cfg       = LaunchConfiguration('rviz_cfg')
    # camera_model   = LaunchConfiguration('camera_model')

    # RTAB-Map parameters
    parameters={
          'frame_id':'zed_camera_link',
        #   'odom_frame_id': 'vo/odom',            # VO uses its own odometry frame to avoid conflicts
          'base_frame_id': 'zed_camera_link',
          'subscribe_stereo': True,
        #   'approx_sync': False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored': True,
        #   'subscribe_odom_info': False,
        #   'wait_for_transform': 1.0,
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '2000',
          'Vis/MinInliers': '10',  # Lower the minimum inlier requirement
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.001',
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings = [
        # Stereo camera data
        # ('odom_rgbd_image', '/stereo_camera/rgbd_image'),
        # ('global_pose', '/zed/zed_node/pose_with_covariance'),
        # ('imu', '/zed/zed_node/imu/data'),
        ('odom', '/vo/odom'),
        ('left/image_rect', '/uoa/cs5917/left/image_rect_color'),
        ('right/image_rect', '/uoa/cs5917/right/image_rect_color'),
        ('left/camera_info', '/zed/zed_node/left/camera_info'), 
        ('right/camera_info', '/zed/zed_node/right/camera_info'),
        # ('delivery_signal', '/uoa/delivery_signal') 
    ]

    auto_delivery_launch = PathJoinSubstitution(
        [get_package_share_directory('uoa_robot_drone'), 'launch', 'auto_delivery.launch.py']
    )
    
    # Path to RViz config (reuse the demo config)
    cfg_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config',
        'demo_robot_mapping.rviz'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time',   default_value='false',
                               description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('localization',   default_value='true',
                               description='Launch in localization mode'),
        DeclareLaunchArgument('rtabmap_viz',    default_value='true',
                               description='Launch RTAB-Map UI'),
        DeclareLaunchArgument('rviz',           default_value='true',
                               description='Launch RViz2'),
        DeclareLaunchArgument('rviz_cfg',       default_value=cfg_rviz,
                               description='Path to RViz2 config file'),
        # DeclareLaunchArgument('camera_model',   default_value='zed2',
        #                        description='ZED camera model for zed_wrapper'),
        
        # 1) Establish TF connection between visual odometry frame and tb3/base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='viz_odom_to_tb3_base_link',
            namespace='tf_publishers',
            arguments=[
                '--frame-id', 'vo/odom',
                '--child-frame-id', 'tb3/base_link',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0'
            ],
            parameters=[{'period': 0.1}],
        ),
        # # Establish TF connection between visual odometry frame and ZED camera link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='viz_odom_to_zed_camera_link',
        #     namespace='tf_publishers',
        #     arguments=[
        #         '--frame-id', 'vo/odom',
        #         '--child-frame-id', 'zed_camera_link',
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '0', '--pitch', '0', '--yaw', '0'
        #     ],
        #     parameters=[{'period': 0.1}],
        # ),

        # 2） Establish TF connection between tb3/base_link and zed_camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tb3_to_zed_camera_transform',
            namespace='tf_publishers',  # Add namespace to avoid conflicts
            # arguments=['0.083', '0', '0.094', '0', '0', '0', 'tb3/base_link', 'zed_camera_link'],
            # arguments: x y z yaw pitch roll parent_frame child_frame
            # Connect tb3/base_link to zed_camera_link, camera is 8.3cm in front and 9.4cm above base_link
            arguments=[
                '--frame_id', 'tb3/base_link',
                '--child_frame_id', 'zed_camera_link',
                '--x', '0.083',
                '--y', '0.0',
                '--z', '0.094',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
            ],
            output='screen',
            parameters=[{'period': 0.1}],  # Publish every 0.1 seconds
        ),
        
        # 3) Establish TF connections for zed_camera_link and zed_left_camera_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_camera_to_left_optical_transform',
            namespace='tf_publishers',
            arguments=[
                '--frame-id', 'zed_camera_link',
                '--child-frame-id', 'zed_left_camera_optical_frame',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'
            ],
            parameters=[{'period': 0.1}],
        ),

        # 4) Establish TF connections for zed_camera_link and zed_right_camera_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_camera_to_right_optical_transform',
            namespace='tf_publishers',
            arguments=[
                '--frame-id', 'zed_camera_link',
                '--child-frame-id', 'zed_right_camera_optical_frame',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'
            ],
            parameters=[{'period': 0.1}],
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='zed_camera_to_depth_optical_transform',
        #     namespace='tf_publishers',
        #     arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'zed_camera_link', 'zed_depth_camera_optical_frame'],
        #     parameters=[{'period': 0.1}],
        # ),
        
        # 5) Left camera image decompression
        Node(
            package='image_transport', 
            executable='republish', 
            name='uoa_pc_repub_left', 
            namespace='uoa',
            output='screen',
            arguments=['compressed', 'raw'],  # compressed -> raw (uncompressed format)
            remappings=[
                ('in/compressed', '/uoa/cs5917/left/image_rect_color/compressed'),
                ('out', '/uoa/cs5917/left/image_rect_color')
            ]
        ),

        # 6) Right camera image decompression
        Node(
            package='image_transport', 
            executable='republish', 
            name='uoa_pc_repub_right', 
            namespace='uoa',  # Use uoa namespace for consistency
            output='screen',
            arguments=['compressed', 'raw'],  # compressed -> raw (uncompressed format)
            remappings=[
                ('in/compressed', '/uoa/cs5917/right/image_rect_color/compressed'),  # Correction: use right camera topic
                ('out', '/uoa/cs5917/right/image_rect_color')
            ]
        ),

        # 1) Convert the image format from the left ZED Camera (bgra8 -> bgr8)
        # Node(
        #     package='convert_image_format', 
        #     executable='convert_image_format',
        #     name='convert_format_left',
        #     namespace='uoa',
        #     output='screen',
        #     parameters=[{
        #         'from_format': 'bgra8',
        #         'to_format': 'bgr8',
        #         'subscription': '/uoa/cs5917/left/image_rect_color',
        #         'publisher': '/uoa/cs5917/left/image_rect_bgr'
        #     }]
        # ),

        # # 2) Convert the image format from the right ZED Camera (bgra8 -> bgr8)
        # Node(
        #     package='convert_image_format', 
        #     executable='convert_image_format',
        #     name='convert_format_right',
        #     namespace='uoa',
        #     output='screen',
        #     parameters=[{
        #         'from_format': 'bgra8',
        #         'to_format': 'bgr8',
        #         'subscription': '/uoa/cs5917/right/image_rect_color',
        #         'publisher': '/uoa/cs5917/right/image_rect_bgr'
        #     }]
        # ),
                        
        # # 3) Synchronize stereo streams
        # Node(
        #     package='rtabmap_sync', executable='stereo_sync', output='screen',
        #     namespace='stereo_camera',
        #     name='stereo_sync',
        #     parameters=[{
        #         'approx_sync': True,                # Disable approximate sync to lower the amount of discarding frames by the stereo_odometry node
        #         'approx_sync_max_interval': 0.05,    # 50ms strict sync to prevent false sync
        #         'topic_queue_size': 100,             # Increase queue buffer
        #         'sync_queue_size': 100,              # Increase sync queue
        #     }],
        #     remappings=[
        #         ('left/image_rect', '/uoa/cs5917/left/image_rect_bgr'),
        #         ('right/image_rect', '/uoa/cs5917/right/image_rect_bgr'),
        #         ('left/camera_info', '/zed/zed_node/left/camera_info'), 
        #         ('right/camera_info', '/zed/zed_node/right/camera_info'),
        #     ]
        # ),

        # # 4) Visual Odometry (VO) node
        # Node(
        #     package='rtabmap_odom', executable='stereo_odometry', output='screen',
        #     parameters=[parameters, {
        #         'approx_sync': True,                # Enable approximate sync
        #         'approx_sync_max_interval': 1.0,  # 1000ms max time difference
        #         'topic_queue_size': 300,
        #         'sync_queue_size': 300,
        #     }],
        #     remappings=remappings
        # ),

        # # 5) SLAM node
        # Node(
        #     condition=UnlessCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings,
        #     arguments=['-d'] # Equivalent to command line: rtabmap -d, it will delete the previous database (~/.ros/rtabmap.db)
        # ),

        # # 6) Localization-only mode
        # Node(
        #     condition=IfCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters, {
        #         'Mem/IncrementalMemory': 'False',
        #         'Mem/InitWMWithAllNodes': 'True'
        #     }],
        #     remappings=remappings
        # ),

        # 7) Delivery parcels automatically (only if localization is true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([auto_delivery_launch]),
            launch_arguments=[
                ('delivery_ready', 'true'),  # Set delivery ready state to true
                ('localization', localization)  # Pass the localization argument
            ]
        ),
      
        # 9) RTAB-Map GUI (optional)
        Node(
            condition=IfCondition(rtabmap_viz),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            name='rtabmap_viz',
            parameters=[parameters],
            remappings=remappings
        ),

        # 10) RViz2 (optional)
        Node(
            condition=IfCondition(rviz),
            package='rviz2', executable='rviz2', output='screen',
            name='rviz2',
            arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]
        ),
    ])
