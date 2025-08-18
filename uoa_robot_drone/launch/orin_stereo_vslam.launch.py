"""
orin_stereo_vslam.launch.py


Pre-requisites:
  Source your ROS2 workspace: `source ~/colcon_ws/install/setup.bash`

Example:
  $ ros2 launch uoa_robot_drone orin_stereo_vslam.launch.py

Launch arguments:
  use_sim_time  – if true, use /clock
  localization  – if true, run in localization-only mode
  rtabmap_viz   – if true, start the RTAB-Map GUI
  rviz          – if true, start RViz2
  rviz_cfg      – path to RViz config
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
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Launch arguments
    use_sim_time   = LaunchConfiguration('use_sim_time')
    localization   = LaunchConfiguration('localization')

    def set_dynamic_detection_rate(context):
        # Copy original parameters dictionary
        params = dict(parameters)
        
        # Get localization parameter value
        loc = context.launch_configurations['localization'].lower()
    
        if loc == 'true':
            params['Rtabmap/DetectionRate'] = '15.0'
        else:
            params['Rtabmap/DetectionRate'] = '5.0'
        nodes = []
        nodes.append(Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[params],
            remappings=remappings,
            arguments=['-d']
        ))
        nodes.append(Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[params, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
            }],
            remappings=remappings
        ))
        return nodes

    # RTAB-Map parameters
    parameters={
            'subscribe_depth': False, # (bool, default: "true") Subscribe to depth image.
            'subscribe_scan': False,  # (bool, default: "false") Subscribe to laser scan.
            'subscribe_stereo': True,  # (bool, default: "false") Subscribe to stereo images. Enable nodes (rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap)) to subscribe to stereo images
            'subscribe_rgb': False, 
            'subscribe_rgbd': False,  # (bool, default: "false") Subscribe to rgbd_image topic, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            'subscribe_odom_info': True,  # Not suitable for rtabmap. True: use internal odometry, False: use external odometry

            'frame_id':'zed_camera_link', # (string, default: "base_link") The frame attached to the mobile base. The frame_id is the coordinate frame's name when the original visual data of the specific sensor is published, involving nodes rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap).
            'gen_scan': True, # (bool, default: "false") Generate laser scans from depth images (using the middle horizontal line of the depth image). Not generated if subscribe_scan or subscribe_scan_cloud are true.
            'gen_scan_max_depth': 4.0, # (double, default: 4.0) Maximum depth of the laser scans generated.
            'approx_sync': False,  # (bool, default: "false") Use approximate time synchronization of input messages. If false, note that the odometry input must have also exactly the same timestamps than the input images. Enable approximate sync for stereo images, involving nodes (rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap))
            'odom_sensor_sync': False,  # (bool, default: "false") Adjust image and scan poses relatively to odometry pose for each node added to graph. For example, if an image received at t=1s has been synchronized with a scan at t=1.1s (and our reference stamp is the scan) and the robot is moving forward at 1 m/s, then we will ask TF to know the motion between t=1s and t=1.1s (which would be 0.1 m) for the camera to adjust its local transform (-10 cm) relative to scan frame at scan stamp. This also applies to multi-camera synchronization.
            'gen_depth_fill_holes_size': 5,  # (int, default: 0) Fill holes of empty pixels up to this size. Values are interpolated from neighbor depth values. 0 means disabled.
            'map_always_update': True,  # (bool, default: "false") Mapping Always update the occupancy grid map even if the graph has not been updated (e.g., by default when the robot is not moving, the graph is not growing). For example, if the environment is dynamic, we may want to update the map even when to robot is not moving. Another approach would be to force rtabmap to always update the graph (by setting RGBD/LinearUpdate to 0), which can be not efficient over time as the map will grow even if the robot doesn't move.
            'map_empty_ray_tracing': True, # (bool, default: "true") Mapping Do ray tracing to fill unknown space for invalid 2D scan's rays (assuming invalid rays to be infinite). Used only when map_always_update is also true.

            'Reg/strategy': '0',  # Registration strategy for loop closure, neighbor link refining, proximity detection, involving node rtabmap_slam(rtabmap). 0: Visual, 1: ICP, 2: Visual + ICP
            'Rtabmap/TimeThr': '2000', # Maximum time (in milliseconds) that RTAB-Map is allowed to spend on processing each frame, mainly involving node rtabmap_slam. Type value: 700
            'Rtabmap/MemoryThr': '2000', # The threshold for the number of nodes retained in the working memory. When the number of nodes in the working memory exceeds this value, RTAB-Map will transfer older nodes to the long-term memory and may extract local representative nodes from it for subsequent loop closure detection, thereby optimizing memory usage, mainly involving node rtabmap_slam.
            'Rtabmap/DetectionRate': '8.0', 
            'map_negative_poses_ignored': True, # Ignore negative poses in the map, mainly involving node rtabmap_slam(rtabmap)
            'qos_image': 1, # Quality of Service, 1：sensor_data（best_effort(no retransmission when missing packages) + volatile(the history is not retained when the node restarting) + keep_last 10(only cache the latest 10 messages)）2：default（reliable(the missing packages will be retransmitted) + volatile + keep_last 10）, mainly involving nodes rtabmap_sync, rtabmap_odom, rtabmap_slam(rtabmap).
            'Grid/RangeMin': '0.0',  # Ignore points closer than this distance, mainly involving node rtabmap_slam(rtabmap).
            'Grid/Sensor': '1',  # 0=lidar, 1=camera/rgbd/pointcloud, 2=both. Mainly involving node rtabmap_slam(rtabmap).
            'Reg/Force3DoF': 'true',  # Force 3 DoF registration, mainly involving node rtabmap_slam(rtabmap).
            'OdomF2M/MaxSize': '5000', # Maximum size of the OdomF2M buffer, mainly involving node rtabmap_odom.
            'Vis/MinInliers': '10',  # (int, default: 20) Minimum number of inliers for visual matching between two images, the current image and the previous one, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            'GFTT/MinDistance': '10', # (double, default: 10) Minimum distance between keypoints detected by GFTT (Good Features to Track), mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
            'GFTT/QualityLevel': '0.001', # (double, default: 0.001) Quality level for GFTT keypoint detection, the higher the value, the fewer keypoints will be detected, mainly involving nodes rtabmap_odom and rtabmap_slam(rtabmap).
    }

    remappings = [
        ('odom', '/vo/odom'), # Internal odometry topic from stereo_odometry node
        ('rgbd_image', '/stereo_camera/rgbd_image'), # From internal stereo sync node
        ('left/image_rect', '/uoa/orin/left/image_rect_bgr'),
        ('right/image_rect', '/uoa/orin/right/image_rect_bgr'),
        ('left/camera_info', '/zed/zed_node/left/camera_info'), 
        ('right/camera_info', '/zed/zed_node/right/camera_info'),
        ('map', '/rtabmap/map'),
    ]

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time',   default_value='false',
                               description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('localization',   default_value='true',
                               description='Launch in localization mode'),
        
        # 1) Convert the image format from the left ZED Camera (bgra8 -> bgr8)
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

        # 2) Convert the image format from the right ZED Camera (bgra8 -> bgr8)
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
           
        # 3) Compress left camera images to reduce bandwidth usage
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
        
        # 4) Compress right camera images to reduce bandwidth usage
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

        # 5) Visual Odometry (VO) node
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings
        ),

        # 6) Dynamically set DetectionRate for SLAM/Localization Node
        OpaqueFunction(function=set_dynamic_detection_rate),
    ])
