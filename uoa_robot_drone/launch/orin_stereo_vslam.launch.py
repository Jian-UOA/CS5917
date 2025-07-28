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
from launch.actions import TimerAction

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
            # 'subscribe_depth': "true" (bool, default: "true") Subscribe to depth image.
            'subscribe_scan': False,  # (bool, default: "false") Subscribe to laser scan.
            # 'subscribe_scan_cloud': "false" (bool, default: "false") Subscribe to laser scan point cloud.
            'subscribe_stereo': True,  # (bool, default: "false") Subscribe to stereo images. Enable nodes (rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap)) to subscribe to stereo images
            'subscribe_rgbd': False,  # (bool, default: "false") Subscribe to rgbd_image topic, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).

            'frame_id':'zed_camera_link', # (string, default: "base_link") The frame attached to the mobile base. The frame_id is the coordinate frame's name when the original visual data of the specific sensor is published, involving nodes rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap).
            # 'map_frame_id': "map" (string, default: "map") The frame attached to the map.
            # 'odom_frame_id': 'vo/odom',  # (string, default: "") The frame attached to odometry. If empty, rtabmap will subscribe to odom topic to get odometry. If set, rtabmap will get odometry from the TF tree (odom_frame_id -> frame_id) and the convariance value is fixed by odom_tf_angular_variance and odom_tf_linear_variance.
            # 'base_frame_id': 'zed_camera_link', # Not suitable for rtabmap. It is not a rtabmap parameter.  The base_frame_id is the coordinate frame's name for the robot's base, involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            # 'visual_odometry': True,  # Not suitable for rtabmap. Enable visual odometry
            # 'icp_odometry': False,  # Not suitable for rtabmap. ICP (Iterative Closest Point) odometry refers to the way of estimating the motion by aligning the closest points between two frames of the point clouds from different sensors iteratively, mainly involving node rtabmap_odom.
            # 'subscribe_odom_info': True,  # Not suitable for rtabmap. True: use internal odometry, False: use external odometry
            # 'subscribe_odom_info': False,  # Not suitable for rtabmap. Use external odometry
            # 'wait_for_transform': True, # (bool, default: "true") Wait (maximum wait_for_transform_duration sec) for transform when a tf transform is not still available. mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap). 
            # wait_for_transform_duration (double, default: 0.1) Wait duration for wait_for_transform.
            'gen_scan': True, # (bool, default: "false") Generate laser scans from depth images (using the middle horizontal line of the depth image). Not generated if subscribe_scan or subscribe_scan_cloud are true.
            'gen_scan_max_depth': 4.0, # (double, default: 4.0) Maximum depth of the laser scans generated.
            'approx_sync': False,  # (bool, default: "false") Use approximate time synchronization of input messages. If false, note that the odometry input must have also exactly the same timestamps than the input images. Enable approximate sync for stereo images, involving nodes (rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap))
            'odom_sensor_sync': False,  # (bool, default: "false") Adjust image and scan poses relatively to odometry pose for each node added to graph. For example, if an image received at t=1s has been synchronized with a scan at t=1.1s (and our reference stamp is the scan) and the robot is moving forward at 1 m/s, then we will ask TF to know the motion between t=1s and t=1.1s (which would be 0.1 m) for the camera to adjust its local transform (-10 cm) relative to scan frame at scan stamp. This also applies to multi-camera synchronization.
            'gen_depth_fill_holes_size': 5,  # (int, default: 0) Fill holes of empty pixels up to this size. Values are interpolated from neighbor depth values. 0 means disabled.
            # 'gen_depth_fill_iterations': 0.1,  # (double, default: 0.1) Number of iterations to fill holes.
            # 'gen_depth_fill_holes_error': 1,  # (int, default: 1) Maximum depth error (m) to interpolate.
            # 'map_filter_radius': 0.0,  # (double, default: 0.0) Mapping Filter nodes before creating the maps. Only load data for one node in the filter radius (the latest data is used) up to filter angle (map_filter_angle). Set to 0.0 to disable node filtering. Used for all published maps.
            # 'map_filter_angle': 30.0,  # (double, default: 30.0) Mapping Angle used when filtering nodes before creating the maps. See also map_filter_radius. Used for all published maps.
            # 'latch': "true",  # (bool, default: "true") Mapping If true, the last message published on the map topics will be saved and sent to new subscribers when they connect.
            'map_always_update': True,  # (bool, default: "false") Mapping Always update the occupancy grid map even if the graph has not been updated (e.g., by default when the robot is not moving, the graph is not growing). For example, if the environment is dynamic, we may want to update the map even when to robot is not moving. Another approach would be to force rtabmap to always update the graph (by setting RGBD/LinearUpdate to 0), which can be not efficient over time as the map will grow even if the robot doesn't move.
            'map_empty_ray_tracing': True, # (bool, default: "true") Mapping Do ray tracing to fill unknown space for invalid 2D scan's rays (assuming invalid rays to be infinite). Used only when map_always_update is also true.

            'Reg/strategy': '0',  # Registration strategy for loop closure, neighbor link refining, proximity detection, involving node rtabmap_slam(rtabmap). 0: Visual, 1: ICP, 2: Visual + ICP
            'Rtabmap/TimeThr': '2000', # Maximum time (in milliseconds) that RTAB-Map is allowed to spend on processing each frame, mainly involving node rtabmap_slam. Type value: 700
            'Rtabmap/MemoryThr': '2000', # The threshold for the number of nodes retained in the working memory. When the number of nodes in the working memory exceeds this value, RTAB-Map will transfer older nodes to the long-term memory and may extract local representative nodes from it for subsequent loop closure detection, thereby optimizing memory usage, mainly involving node rtabmap_slam.
            'map_negative_poses_ignored': True, # Ignore negative poses in the map, mainly involving node rtabmap_slam(rtabmap)
            'qos_image': 1, # Quality of Service, 1：sensor_data（best_effort(no retransmission when missing packages) + volatile(the history is not retained when the node restarting) + keep_last 10(only cache the latest 10 messages)）2：default（reliable(the missing packages will be retransmitted) + volatile + keep_last 10）, mainly involving nodes rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap).
            'Grid/RangeMin': '0.0',  # Ignore points closer than this distance, mainly involving node rtabmap_slam(rtabmap).
            'Grid/Sensor': '1',  # 0=lidar, 1=camera/rgbd/pointcloud, 2=both. Mainly involving node rtabmap_slam(rtabmap).
            'Reg/Force3DoF': 'true',  # Force 3 DoF registration, mainly involving node rtabmap_slam(rtabmap).
            # RTAB-Map's internal parameters should be strings
            'OdomF2M/MaxSize': '5000', # Maximum size of the OdomF2M buffer, mainly involving node rtabmap_odom.
            'Vis/MinInliers': '10',  # Minimum number of inliers for visual matching between two images, the current image and the previous one, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            'GFTT/MinDistance': '10', # Minimum distance between keypoints detected by GFTT (Good Features to Track), mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            'GFTT/QualityLevel': '0.001', # Quality level for GFTT keypoint detection, the higher the value, the fewer keypoints will be detected, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
            #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings = [
        # Stereo camera data
        # ('odom_rgbd_image', '/stereo_camera/rgbd_image'),
        # ('global_pose', '/zed/zed_node/pose_with_covariance'),
        # ('imu', '/zed/zed_node/imu/data'),
        # ('odom', '/vo/odom'), # Internal odometry topic from stereo_odometry node
        ('odom', '/zed/zed_node/odom'), # External odometry topic
        # ('rgbd_image', '/stereo_camera/rgbd_image'), # From internal stereo sync node
        ('left/image_rect', '/uoa/orin/left/image_rect_bgr'),
        ('right/image_rect', '/uoa/orin/right/image_rect_bgr'),
        ('left/camera_info', '/zed/zed_node/left/camera_info'), 
        ('right/camera_info', '/zed/zed_node/right/camera_info'),
        # ('delivery_signal', '/uoa/delivery_signal') 
    ]
    


    # Path to RViz config (reuse the demo config)
    cfg_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config',
        'demo_robot_mapping.rviz'
    )

    # Include ZED Wrapper launch
    zed_wrapper_launch = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    ])
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time',   default_value='false',
                               description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('localization',   default_value='true',
                               description='Launch in localization mode'),
        DeclareLaunchArgument('rtabmap_viz',    default_value='false',
                               description='Launch RTAB-Map UI'),
        DeclareLaunchArgument('rviz',           default_value='false',
                               description='Launch RViz2'),
        DeclareLaunchArgument('rviz_cfg',       default_value=cfg_rviz,
                               description='Path to RViz2 config file'),
        # DeclareLaunchArgument('camera_model',   default_value='zed2',
        #                        description='ZED camera model for zed_wrapper'),

        # # 1) Include ZED Wrapper launch with camera model argument
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(zed_wrapper_launch),
        #     launch_arguments=[('camera_model', camera_model)]
        # ),
        
        # # 2) Establish TF connection between TurtleBot3 and ZED camera
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tb3_to_zed_camera_transform',
        #     namespace='tf_publishers',  # Add namespace to avoid conflicts
        #     arguments=['0.083', '0', '0.094', '0', '0', '0', 'tb3/base_link', 'zed_camera_link'],
        #     # arguments: x y z yaw pitch roll parent_frame child_frame
        #     # Connect tb3/base_link to zed_camera_link, camera is 8.3cm in front and 9.4cm above base_link
        #     parameters=[{'period': 0.1}],  # Publish every 0.1 seconds
        # ),
        #=== All ZED related static_transform_publisher nodes to ensure TF is published regularly ===
        
        # # 3) Establish TF connection between visual odometry frame and ZED camera link
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

        # # 4) Establish TF connections for zed_camera_link and zed_left_camera_optical_frame
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='zed_camera_to_left_optical_transform',
        #     namespace='tf_publishers',
        #     arguments=[
        #         '--frame-id', 'zed_camera_link',
        #         '--child-frame-id', 'zed_left_camera_optical_frame',
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'
        #     ],
        #     parameters=[{'period': 0.1}],
        # ),

        # # 5) Establish TF connections for zed_camera_link and zed_right_camera_optical_frame
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='zed_camera_to_right_optical_transform',
        #     namespace='tf_publishers',
        #     arguments=[
        #         '--frame-id', 'zed_camera_link',
        #         '--child-frame-id', 'zed_right_camera_optical_frame',
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'
        #     ],
        #     parameters=[{'period': 0.1}],
        # ),

        # 6) Establish TF connections for zed_camera_link and zed_depth_camera_optical_frame
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='zed_camera_to_depth_optical_transform',
        #     namespace='tf_publishers',
        #     arguments=[
        #         '--frame-id', 'zed_camera_link',
        #         '--child-frame-id', 'zed_depth_camera_optical_frame',
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'
        #     ],
        #     parameters=[{'period': 0.1}],
        # ),

        # 7) Convert the image format from the left ZED Camera (bgra8 -> bgr8)
        Node(
            package='convert_image_format', 
            executable='convert_image_format',
            name='convert_format_left',
            namespace='uoa',
            output='screen',
            parameters=[{
                'from_format': 'bgra8',
                'to_format': 'bgr8',
                'subscription': '/zed/zed_node/left/image_rect_color',
                'publisher': '/uoa/orin/left/image_rect_bgr'
            }]
        ),

        # 8) Convert the image format from the right ZED Camera (bgra8 -> bgr8)
        Node(
            package='convert_image_format', 
            executable='convert_image_format',
            name='convert_format_right',
            namespace='uoa',
            output='screen',
            parameters=[{
                'from_format': 'bgra8',
                'to_format': 'bgr8',
                'subscription': '/zed/zed_node/right/image_rect_color',
                'publisher': '/uoa/orin/right/image_rect_bgr'
            }]
        ),
           
        # 9) Compress left camera images to reduce bandwidth usage
        Node(
            package='image_transport', 
            executable='republish', 
            name='orin_repub_left', 
            namespace='uoa',
            output='screen',
            arguments=['raw', 'compressed'],  # raw -> compressed (compressed format)
            remappings=[
                ('in', '/uoa/orin/left/image_rect_bgr'),
                ('out/compressed', '/uoa/cs5917/left/image_rect_color/compressed')
            ]
        ),
        
        # 10) Compress right camera images to reduce bandwidth usage
        Node(
            package='image_transport', 
            executable='republish', 
            name='orin_repub_right', 
            namespace='uoa',  # Use uoa namespace for consistency
            output='screen',
            arguments=['raw', 'compressed'],  # raw -> compressed (compressed format)
            remappings=[
                ('in', '/uoa/orin/right/image_rect_bgr'),  
                ('out/compressed', '/uoa/cs5917/right/image_rect_color/compressed')
            ]
        ),

        # # 11) Synchronize stereo streams
        # Node(
        #     package='rtabmap_sync', executable='stereo_sync', output='screen',
        #     namespace='stereo_camera',
        #     name='stereo_sync',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),

        # 12) Visual Odometry (VO) node
        # Node(
        #     package='rtabmap_odom', executable='stereo_odometry', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),

        # 13) SLAM node
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'] # Equivalent to command line: rtabmap -d, it will delete the previous database (~/.ros/rtabmap.db)
        ),

        # 14) Localization-only mode
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
                'Rtabmap/DetectionRate': '5.0',  # Frequency of publishing the topic /localization_pose
            }],
            remappings=remappings
        ),

        # 15) Listen to robot position and publish delivery signal
    #     Node(
    #         package='uoa_robot_drone', executable='robot_position_listener', output='screen',
    #         namespace='uoa',
    #         name='robot_position_listener',
    #         # Use parameters to set goal position and orientation
    #         parameters=[{
    #             'goal_position_x': 1.2,
    #             'goal_position_y': 1.4,
    #             'goal_position_z': 0.0,
    #             'goal_orientation_x': 0.0,
    #             'goal_orientation_y': 0.0,
    #             'goal_orientation_z': 3.14/2,  # 90 degrees in radians
    #             'goal_orientation_w': 0.2,
    #             'distance_threshold': 0.1,
    #             'orientation_threshold': 0.1
    #         }],
    #         remappings=remappings
    #     ),

    #     # 16) Delivery parcels automatically (only if localization is true)
    #     Node(
    #         condition=IfCondition(localization),
    #         package='uoa_robot_drone', executable='auto_delivery', output='screen',
    #         namespace='uoa',
    #         name='auto_delivery',
    #         remappings=remappings
    #     ),

    #     # 17) RTAB-Map GUI (optional)
    #     Node(
    #         condition=IfCondition(rtabmap_viz),
    #         package='rtabmap_viz', executable='rtabmap_viz', output='screen',
    #         parameters=[parameters],
    #         remappings=remappings
    #     ),

    #     # 18) RViz2 (optional)
    #     Node(
    #         condition=IfCondition(rviz),
    #         package='rviz2', executable='rviz2', name='rviz2', output='screen',
    #         arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]
    #     ),
    ])
