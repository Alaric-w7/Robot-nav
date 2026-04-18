import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
    elderbot_base_dir = get_package_share_directory('elderbot_base')
    elderbot_navigation_dir = get_package_share_directory('elderbot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(elderbot_navigation_dir, 'config', 'slam.yaml')
    nav2_params_file = os.path.join(elderbot_navigation_dir, 'config', 'navigation.yaml')
    explore_params_file = os.path.join(elderbot_navigation_dir, 'config', 'explore.yaml')

    imu_topic = LaunchConfiguration('imu_topic')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    # Hardware components
    can_driver_node = Node(
        package='elderbot_bringup',
        executable='can_driver',
        name='can_driver',
        output='screen'
    )

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
        parameters=[{'range_min_filter': 0.20}]
    )

    camera_launch = create_ascamera_driver_launch()
    ascamera_tf_nodes = create_ascamera_tf_nodes()
    depth_scan_nodes = create_depth_scan_pipeline(scan_height=40)
    imu_node = create_imu_node()

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'imu0': imu_topic}],
        remappings=[("odometry/filtered", "/odom")]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_config_path
        }.items()
    )

    # Nav2 pure navigation (no map_server/amcl)
    nav2_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'False',
                    'autostart': 'True',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    # Explore Lite Node
    explore_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[explore_params_file]
            )
        ]
    )

    return LaunchDescription([
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
        slam_toolbox_launch,
        nav2_launch,
        explore_node
    ])
