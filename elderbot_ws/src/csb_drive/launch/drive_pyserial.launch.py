from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    log_raw_frames = LaunchConfiguration("log_raw_frames")
    publish_scan = LaunchConfiguration("publish_scan")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("csb_drive"), "config", "dyp_e08.yaml"]
                ),
                description="Path to the ultrasonic driver parameter file.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Serial device path for the ultrasonic radar.",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value="9600",
                description="Serial baudrate.",
            ),
            DeclareLaunchArgument(
                "log_raw_frames",
                default_value="false",
                description="Whether to print raw UART frames.",
            ),
            DeclareLaunchArgument(
                "publish_scan",
                default_value="true",
                description="Whether to publish /scan_ultrasonic.",
            ),
            Node(
                package="csb_drive",
                executable="drive_pyserial",
                name="ultrasonic_driver",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "port": port,
                        "baudrate": ParameterValue(baudrate, value_type=int),
                        "log_raw_frames": ParameterValue(
                            log_raw_frames, value_type=bool
                        ),
                        "publish_scan": ParameterValue(
                            publish_scan, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
