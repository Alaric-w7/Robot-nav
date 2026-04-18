import atexit
import fcntl
import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


_BRINGUP_LOCK_HANDLE = None


def _get_bringup_lock_path():
    runtime_dir = os.environ.get('XDG_RUNTIME_DIR')
    if runtime_dir and os.path.isdir(runtime_dir) and os.access(runtime_dir, os.W_OK):
        return os.path.join(runtime_dir, f'elderbot_bringup_{os.getuid()}.lock')
    return f'/tmp/elderbot_bringup_{os.getuid()}.lock'


def _release_bringup_lock():
    global _BRINGUP_LOCK_HANDLE

    if _BRINGUP_LOCK_HANDLE is None:
        return

    try:
        fcntl.flock(_BRINGUP_LOCK_HANDLE.fileno(), fcntl.LOCK_UN)
    except OSError:
        pass

    _BRINGUP_LOCK_HANDLE.close()
    _BRINGUP_LOCK_HANDLE = None


def _acquire_bringup_lock():
    global _BRINGUP_LOCK_HANDLE

    if _BRINGUP_LOCK_HANDLE is not None:
        return

    lock_path = _get_bringup_lock_path()
    lock_handle = open(lock_path, 'w', encoding='utf-8')

    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_handle.close()
        raise RuntimeError(
            'elderbot_bringup is already running. Stop the existing launch before starting another one.'
        ) from exc

    lock_handle.write(f'{os.getpid()}\n')
    lock_handle.flush()
    _BRINGUP_LOCK_HANDLE = lock_handle
    atexit.register(_release_bringup_lock)


def generate_launch_description():
    _acquire_bringup_lock()
    home_dir = os.path.expanduser('~')

    elderbot_base_dir = get_package_share_directory('elderbot_base')
    elderbot_navigation_dir = get_package_share_directory('elderbot_navigation')
    csb_drive_dir = get_package_share_directory('csb_drive')

    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(elderbot_navigation_dir, 'config', 'navigation.yaml')
    ultrasonic_params_file = os.path.join(csb_drive_dir, 'config', 'dyp_e08.yaml')

    map_file = LaunchConfiguration('map')
    camera_imu_topic = LaunchConfiguration('camera_imu_topic')
    launch_ultrasonic = LaunchConfiguration('launch_ultrasonic')
    ultrasonic_port = LaunchConfiguration('ultrasonic_port')
    ultrasonic_baudrate = LaunchConfiguration('ultrasonic_baudrate')
    ultrasonic_log_raw_frames = LaunchConfiguration('ultrasonic_log_raw_frames')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ================= Nodes =================

    def ultrasonic_tf_node(name, x, y, z, yaw_deg):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{name}_static_tf',
            output='screen',
            condition=IfCondition(launch_ultrasonic),
            arguments=[
                '--x', f'{x}',
                '--y', f'{y}',
                '--z', f'{z}',
                '--yaw', f'{math.radians(yaw_deg)}',
                '--frame-id', 'base_link',
                '--child-frame-id', name,
            ]
        )

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
        output='screen'
    )

    initial_pose_node = Node(
        package='elderbot_bringup',
        executable='initial_pose_pub',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': ParameterValue(initial_pose_x, value_type=float),
            'y': ParameterValue(initial_pose_y, value_type=float),
            'yaw': ParameterValue(initial_pose_yaw, value_type=float),
        }]
    )

    ultrasonic_driver_node = Node(
        package='csb_drive',
        executable='drive_pyserial',
        name='ultrasonic_driver',
        output='screen',
        condition=IfCondition(launch_ultrasonic),
        parameters=[
            ultrasonic_params_file,
            {
                'port': ultrasonic_port,
                'baudrate': ParameterValue(ultrasonic_baudrate, value_type=int),
                'log_raw_frames': ParameterValue(ultrasonic_log_raw_frames, value_type=bool),
            }
        ]
    )

    ultrasonic_tf_nodes = [
        ultrasonic_tf_node('ultrasonic_1', 0.10, 0.17, 0.38, 60.0),
        ultrasonic_tf_node('ultrasonic_2', 0.17, 0.10, 0.38, 30.0),
        ultrasonic_tf_node('ultrasonic_3', 0.17, -0.10, 0.38, -30.0),
        ultrasonic_tf_node('ultrasonic_4', 0.10, -0.17, 0.38, -60.0),
    ]

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
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x in map frame'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y in map frame'
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw in radians'
        ),
        DeclareLaunchArgument(
            'launch_ultrasonic',
            default_value='true',
            description='Whether to launch the csb_drive ultrasonic driver'
        ),
        DeclareLaunchArgument(
            'ultrasonic_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial device path for the ultrasonic radar'
        ),
        DeclareLaunchArgument(
            'ultrasonic_baudrate',
            default_value='9600',
            description='Serial baudrate for the ultrasonic radar'
        ),
        DeclareLaunchArgument(
            'ultrasonic_log_raw_frames',
            default_value='false',
            description='Whether to print raw ultrasonic UART frames'
        ),
        can_driver_node,
        description_launch,
        rplidar_launch,
        laser_filter_node,
        camera_launch,
        depth_to_scan_node,
        depth_filter_node,
        ekf_node,
        initial_pose_node,
        auto_dock_node,
        ultrasonic_driver_node,
        *ultrasonic_tf_nodes,
        nav2_launch
    ])
