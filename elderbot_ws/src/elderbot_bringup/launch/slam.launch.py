import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(elderbot_navigation_dir, 'config', 'slam.yaml')

    imu_topic = LaunchConfiguration('imu_topic')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    return LaunchDescription([
        *get_ascamera_launch_arguments(),

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

        create_ascamera_driver_launch(),
        *create_ascamera_tf_nodes(),
        *create_depth_scan_pipeline(scan_height=40),
        create_imu_node(),

        # 5. EKF sensor fusion (linorobot2 style)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'imu0': imu_topic}],
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
