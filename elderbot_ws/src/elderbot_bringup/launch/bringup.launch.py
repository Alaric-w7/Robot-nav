import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from elderbot_bringup.ascamera_pipeline import (
    create_ascamera_driver_launch,
    create_ascamera_tf_nodes,
    create_depth_scan_pipeline,
    create_imu_node,
    get_ascamera_launch_arguments,
)


def generate_launch_description():
    home_dir = os.path.expanduser('~')

    elderbot_base_dir = get_package_share_directory('elderbot_base')
    elderbot_navigation_dir = get_package_share_directory('elderbot_navigation')

    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(elderbot_navigation_dir, 'config', 'navigation.yaml')

    map_file = LaunchConfiguration('map')
    imu_topic = LaunchConfiguration('imu_topic')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
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

    camera_launch = create_ascamera_driver_launch()
    ascamera_tf_nodes = create_ascamera_tf_nodes()
    depth_scan_nodes = create_depth_scan_pipeline(scan_height=200)
    imu_node = create_imu_node()

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'imu0': imu_topic}],
        remappings=[("odometry/filtered", "/odom")]
    )

    initial_pose_node = Node(
        package='elderbot_bringup',
        executable='initial_pose_pub',
        name='initial_pose_pub',
        output='screen',
        condition=IfCondition(publish_initial_pose),
        parameters=[{
            'x': initial_pose_x,
            'y': initial_pose_y,
            'yaw': initial_pose_yaw,
            'delay_sec': 4.0,
            'repeat_count': 5,
        }]
    )

    auto_dock_node = Node(
        package='elderbot_navigation',
        executable='auto_dock_node.py',
        name='auto_dock_node',
        output='screen'
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
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x published to /initialpose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y published to /initialpose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw published to /initialpose.',
        ),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='false',
            description='Publish the configured initial pose to /initialpose.',
        ),
        *get_ascamera_launch_arguments(),
        can_driver_node,
        description_launch,
        rplidar_launch,
        laser_filter_node,
        camera_launch,
        *ascamera_tf_nodes,
        *depth_scan_nodes,
        imu_node,
        ekf_node,
        initial_pose_node,
        auto_dock_node,
        nav2_launch
    ])
