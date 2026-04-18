import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("elderbot_description"), "urdf/robots", "elderbot.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),
        DeclareLaunchArgument(
            name='publish_joints',
            default_value='true',
            description='Launch joint_states_publisher'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            additional_env=get_rviz_compat_env()
        )
    ])
