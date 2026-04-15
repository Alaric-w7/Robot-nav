import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    elderbot_base_dir = get_package_share_directory('elderbot_base')
    elderbot_navigation_dir = get_package_share_directory('elderbot_navigation')

    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(elderbot_navigation_dir, 'config', 'slam.yaml')

    camera_imu_topic = LaunchConfiguration('camera_imu_topic')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='IMU topic published by depth camera'
        ),

        # 1. CAN driver (hardware-specific, must keep)
        Node(
            package='elderbot_bringup',
            executable='can_driver',
            name='can_driver',
            output='screen'
        ),

        # 2. Robot description via URDF (linorobot2 style)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        # 3. RPLidar C1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
            ),
            launch_arguments={'serial_port': '/dev/rplidar'}.items()
        ),

        # 3.5 Laser filter (filter out lidar mounting poles)
        Node(
            package='elderbot_bringup',
            executable='laser_filter',
            name='laser_filter',
            output='screen',
            parameters=[{'range_min_filter': 0.20}]
        ),

        # 4. Depth camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_camera_dir, 'launch', 'gemini_330_series.launch.py')
            ),
            launch_arguments={
                'enable_point_cloud': 'false',
                'depth_registration': 'false',
                'enable_colored_point_cloud': 'false',
                'enable_left_ir': 'false',
                'enable_right_ir': 'false',
                'time_domain': 'system',
                'log_level': 'info',
            }.items()
        ),

        # 4.5 Convert depth image to virtual LaserScan for obstacle detection
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'scan_time': 0.033,
                'range_min': 0.05,
                'range_max': 5.0,
                'scan_height': 40,
                'output_frame': 'camera_link',
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan_depth_raw'),
            ]
        ),

        Node(
            package='elderbot_bringup',
            executable='depth_scan_filter',
            name='depth_scan_filter',
            output='screen'
        ),

        # 5. EKF sensor fusion (linorobot2 style)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'imu0': camera_imu_topic}],
            remappings=[("odometry/filtered", "/odom")]
        ),

        # 6. SLAM Toolbox (linorobot2 style)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_config_path
            }.items()
        ),
    ])
