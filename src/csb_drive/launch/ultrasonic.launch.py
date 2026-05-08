from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('csb_drive'), 'config', 'ultrasonic.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for the DYP-E08 RS485 ultrasonic controller'
        ),
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='YAML parameter file for the ultrasonic driver'
        ),
        Node(
            package='csb_drive',
            executable='drive_pyserial',
            name='dyp_e08_pyserial_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'port': LaunchConfiguration('port')},
            ],
        ),
    ])
