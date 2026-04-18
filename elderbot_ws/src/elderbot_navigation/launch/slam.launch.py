import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_rviz_compat_env():
    if os.uname().machine != 'aarch64':
        return {}

    rockchip_dri = '/usr/lib/aarch64-linux-gnu/dri/rockchip_dri.so'
    mali_egl = '/usr/lib/aarch64-linux-gnu/mali/libEGL.so.1'
    if not (os.path.exists(rockchip_dri) and os.path.exists(mali_egl)):
        return {}

    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    mesa_path = '/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu'
    if ld_library_path:
        ld_library_path = f'{mesa_path}:{ld_library_path}'
    else:
        ld_library_path = mesa_path

    env = {
        'LD_LIBRARY_PATH': ld_library_path,
        'QT_OPENGL': 'desktop',
        'QT_XCB_GL_INTEGRATION': 'xcb_glx',
        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
        'LIBGL_DRI3_DISABLE': '1',
    }
    return env


def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_navigation'), 'config', 'slam.yaml']
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_navigation'), 'config', 'navigation.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_navigation'), 'rviz', 'slam.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sim_time to true'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'slam_params_file': slam_config_path
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}],
            additional_env=get_rviz_compat_env()
        )
    ])
