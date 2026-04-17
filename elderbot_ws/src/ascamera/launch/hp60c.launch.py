from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIG_PATH = "/home/ztl/Robot-nav/elderbot_ws/src/ascamera/configurationfiles"


def create_hp60c_node(namespace, usb_bus_no, usb_path, condition=None):
    return Node(
        namespace=namespace,
        package="ascamera",
        executable="ascamera_node",
        respawn=True,
        output="both",
        condition=condition,
        parameters=[
            {"usb_bus_no": usb_bus_no},
            {"usb_path": usb_path},
            {"confiPath": CONFIG_PATH},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        remappings=[],
    )


def generate_launch_description():
    camera_1_usb_bus_no_arg = DeclareLaunchArgument(
        "camera_1_usb_bus_no",
        default_value="7",
        description="Fixed USB bus number for the first HP60C camera.",
    )
    camera_1_usb_path_arg = DeclareLaunchArgument(
        "camera_1_usb_path",
        default_value="1.1.1",
        description="Fixed USB port path for the first HP60C camera.",
    )
    camera_2_enabled_arg = DeclareLaunchArgument(
        "camera_2_enabled",
        default_value="false",
        description="Enable the second HP60C camera node after its USB bus/path are confirmed.",
    )
    camera_2_usb_bus_no_arg = DeclareLaunchArgument(
        "camera_2_usb_bus_no",
        default_value="7",
        description="Fixed USB bus number for the second HP60C camera.",
    )
    camera_2_usb_path_arg = DeclareLaunchArgument(
        "camera_2_usb_path",
        default_value="set_me",
        description="Fixed USB port path for the second HP60C camera.",
    )

    camera_1_node = create_hp60c_node(
        namespace="ascamera_hp60c",
        usb_bus_no=ParameterValue(LaunchConfiguration("camera_1_usb_bus_no"), value_type=int),
        usb_path=LaunchConfiguration("camera_1_usb_path"),
    )

    camera_2_node = create_hp60c_node(
        namespace="ascamera_hp60c_2",
        usb_bus_no=ParameterValue(LaunchConfiguration("camera_2_usb_bus_no"), value_type=int),
        usb_path=LaunchConfiguration("camera_2_usb_path"),
        condition=IfCondition(LaunchConfiguration("camera_2_enabled")),
    )

    return LaunchDescription([
        camera_1_usb_bus_no_arg,
        camera_1_usb_path_arg,
        camera_2_enabled_arg,
        camera_2_usb_bus_no_arg,
        camera_2_usb_path_arg,
        camera_1_node,
        camera_2_node,
    ])
