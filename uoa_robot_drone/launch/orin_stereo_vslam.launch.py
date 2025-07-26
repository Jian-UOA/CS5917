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
            'frame_id':'zed_camera_link',
            'odom_frame_id': 'vo/odom',            # VO uses its own odometry frame to avoid conflicts
            'base_frame_id': 'zed_camera_link',
            'visual_odometry': True,  # Enable visual odometry
            'icp_odometry': False,  # Disable ICP (Iterative Closest Point) odometry
            'stereo': True,  # Enable stereo processing
            'subscribe_rgbd': False,  # Subscribe to RGBD images
            'subscribe_stereo': True,  # Disable stereo subscription since we are using RGBD images
            'subscribe_odom_info': True,  # True: use internal odometry, False: use external odometry
            'subscribe_scan': False,  # Disable scan subscription
            'approx_sync': True,  # Enable approximate sync for stereo images
            'Reg/strategy': '0',  # Registration strategy for loop closure, neighbor link refining, proximity detection. 0: Visual, 1: ICP, 2: Visual + ICP
            'Rtabmap/TimeThr': '2000', # Maximum time (in milliseconds) that RTAB-Map is allowed to spend on processing each frame. Type value: 700
            'Rtabmap/MemoryThr': '2000', # The threshold for the number of nodes retained in the working memory. When the number of nodes in the working memory exceeds this value, RTAB-Map will transfer older nodes to the long-term memory and may extract local representative nodes from it for subsequent loop closure detection, thereby optimizing memory usage.
            'map_negative_poses_ignored': True,
            'qos_image': 1, # 1：sensor_data（best_effort + volatile + keep_last 10）2：default（reliable + volatile + keep_last 10）
            'Grid/RangeMin': '0.2',  # Minimum range for grid map
            'Grid/Sensor': '1',  # 0=lidar, 1=camera/rgbd/pointcloud, 2=both.
            'Reg/Force3DoF': 'true',  # Force 3 DoF registration
            'wait_for_transform': 0.5, # Wait for transform to be available before processing images
            # RTAB-Map's internal parameters should be strings
            'OdomF2M/MaxSize': '5000',
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
        # ('odom', '/zed/zed_node/odom'), # External odometry topic
        ('rgbd_image', '/stereo_camera/rgbd_image'), # From internal stereo sync node
        ('odom', '/vo/odom'), # Internal odometry topic from stereo_odometry node
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

        # 11) Synchronize stereo streams
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            name='stereo_sync',
            parameters=[parameters],
            remappings=remappings
        ),

        # 12) Visual Odometry (VO) node
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings
        ),

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
