import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home_dir = os.path.expanduser('~')

    elderbot_base_dir = get_package_share_directory('elderbot_base')
    elderbot_navigation_dir = get_package_share_directory('elderbot_navigation')

    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(elderbot_navigation_dir, 'config', 'navigation.yaml')

    map_file = LaunchConfiguration('map')
    camera_imu_topic = LaunchConfiguration('camera_imu_topic')
    enable_auto_dock = LaunchConfiguration('enable_auto_dock')
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic')
    ultrasonic_port = LaunchConfiguration('ultrasonic_port')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ================= Nodes =================

    can_driver_node = Node(
        package='elderbot_bringup',
        executable='can_driver',
        name='can_driver',
        output='screen'
    )

    # 电池监控已集成到 can_driver 中，无需单独节点

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    laser_filter_node = Node(
        package='elderbot_bringup',
        executable='laser_filter',
        name='laser_filter',
        output='screen',
        parameters=[{
            'range_min_filter': 0.16,
            'rear_blind_angle': 90.0,  # 过滤后方90度, 保留前方270度
        }]
    )

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('csb_drive'), 'launch', 'ultrasonic.launch.py'])
        ),
        launch_arguments={'port': ultrasonic_port}.items(),
        condition=IfCondition(enable_ultrasonic)
    )

    ultrasonic_memory_node = Node(
        package='elderbot_bringup',
        executable='ultrasonic_memory',
        name='ultrasonic_memory',
        output='screen',
        condition=IfCondition(enable_ultrasonic),
        parameters=[{
            'scan_topic': '/scan_ultrasonic',
            'cloud_topic': '/ultrasonic_memory_cloud',
            'target_frame': 'map',
            'memory_duration': 180.0,
            'mark_max_range': 1.0,
            'grid_resolution': 0.08,
            'min_hits': 8,
            'min_confirm_age': 0.8,
            'clear_enabled': True,
            'clear_max_range': 1.2,
            'clear_margin': 0.12,
            'clear_hits': 5,
            'clear_step': 0.04,
            'publish_rate': 2.0,
            'point_z': 0.25,
        }]
    )

    camera_launch = IncludeLaunchDescription(
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
    )

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.05,
            'range_max': 5.0,
            'scan_height': 200,
            'output_frame': 'camera_link',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_depth_raw'),
        ]
    )

    depth_filter_node = Node(
        package='elderbot_bringup',
        executable='depth_scan_filter',
        name='depth_scan_filter',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'imu0': camera_imu_topic}],
        remappings=[("odometry/filtered", "/odom")]
    )

    auto_dock_node = Node(
        package='elderbot_navigation',
        executable='auto_dock_node.py',
        name='auto_dock_node',
        output='screen',
        condition=IfCondition(enable_auto_dock)
    )

    nav2_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'False',
                    'autostart': 'True',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    # ================= Launch =================
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(home_dir, 'maps', 'my_map1.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'camera_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='IMU topic published by depth camera'
        ),
        DeclareLaunchArgument(
            'enable_auto_dock',
            default_value='false',
            description='Whether to start auto_dock_node during navigation bringup'
        ),
        DeclareLaunchArgument(
            'enable_ultrasonic',
            default_value='true',
            description='Whether to start the front ultrasonic sensors'
        ),
        DeclareLaunchArgument(
            'ultrasonic_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for the DYP-E08 RS485 ultrasonic controller'
        ),
        can_driver_node,
        description_launch,
        rplidar_launch,
        laser_filter_node,
        ultrasonic_launch,
        ultrasonic_memory_node,
        camera_launch,
        depth_to_scan_node,
        depth_filter_node,
        ekf_node,
        auto_dock_node,
        nav2_launch
    ])
