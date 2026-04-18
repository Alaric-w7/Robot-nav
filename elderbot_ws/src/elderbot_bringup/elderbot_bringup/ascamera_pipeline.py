import math
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ASCAMERA_CAMERA_1_NAMESPACE = 'ascamera_hp60c'
ASCAMERA_CAMERA_2_NAMESPACE = 'ascamera_hp60c_2'

ASCAMERA_CAMERA_1_DEPTH_IMAGE = f'/{ASCAMERA_CAMERA_1_NAMESPACE}/camera_publisher/depth0/image_raw'
ASCAMERA_CAMERA_1_DEPTH_INFO = f'/{ASCAMERA_CAMERA_1_NAMESPACE}/camera_publisher/depth0/camera_info'
ASCAMERA_CAMERA_2_DEPTH_IMAGE = f'/{ASCAMERA_CAMERA_2_NAMESPACE}/camera_publisher/depth0/image_raw'
ASCAMERA_CAMERA_2_DEPTH_INFO = f'/{ASCAMERA_CAMERA_2_NAMESPACE}/camera_publisher/depth0/camera_info'

ASCAMERA_CAMERA_1_SCAN_RAW = '/scan_depth/raw/camera_1'
ASCAMERA_CAMERA_1_SCAN = '/scan_depth/camera_1'
ASCAMERA_CAMERA_2_SCAN_RAW = '/scan_depth/raw/camera_2'
ASCAMERA_CAMERA_2_SCAN = '/scan_depth/camera_2'
ASCAMERA_CAMERA_1_SCAN_FRAME = f'{ASCAMERA_CAMERA_1_NAMESPACE}_camera_link_0'
ASCAMERA_CAMERA_2_SCAN_FRAME = f'{ASCAMERA_CAMERA_2_NAMESPACE}_camera_link_0'

# In ROS camera_link (x forward, y left, z up), tilting the camera upward is a negative pitch.
DEFAULT_CAMERA_UPWARD_PITCH_RAD = str(-math.radians(40.0))


def get_ascamera_launch_arguments():
    return [
        DeclareLaunchArgument(
            'launch_ascamera',
            default_value='true',
            description='Launch the dual HP60C ascamera driver inside bringup.',
        ),
        DeclareLaunchArgument(
            'camera_1_usb_bus_no',
            default_value='7',
            description='USB bus number for ascamera camera 1 when launch_ascamera is true.',
        ),
        DeclareLaunchArgument(
            'camera_1_usb_path',
            default_value='1.1.1',
            description='USB path for ascamera camera 1 when launch_ascamera is true.',
        ),
        DeclareLaunchArgument(
            'camera_2_enabled',
            default_value='false',
            description='Enable the second ascamera depth pipeline.',
        ),
        DeclareLaunchArgument(
            'camera_2_usb_bus_no',
            default_value='7',
            description='USB bus number for ascamera camera 2 when launch_ascamera is true.',
        ),
        DeclareLaunchArgument(
            'camera_2_usb_path',
            default_value='set_me',
            description='USB path for ascamera camera 2 when launch_ascamera is true.',
        ),
        DeclareLaunchArgument(
            'camera_1_pose_x',
            default_value='0.20',
            description='Camera 1 mount x offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_1_pose_y',
            default_value='-0.14',
            description='Camera 1 mount y offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_1_pose_z',
            default_value='0.34',
            description='Camera 1 mount z offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_1_roll',
            default_value='0.0',
            description='Camera 1 roll in radians.',
        ),
        DeclareLaunchArgument(
            'camera_1_pitch',
            default_value=DEFAULT_CAMERA_UPWARD_PITCH_RAD,
            description='Camera 1 pitch in radians. Default is 40 degrees upward.',
        ),
        DeclareLaunchArgument(
            'camera_1_yaw',
            default_value='-0.57',
            description='Camera 1 yaw in radians.绕z轴旋转0.57弧度（约32.7度）使相机向左偏转，以覆盖机器人前方更宽的区域。',
        ),
        DeclareLaunchArgument(
            'camera_2_pose_x',
            default_value='0.2',
            description='Camera 2 mount x offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_2_pose_y',
            default_value='0.14',
            description='Camera 2 mount y offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_2_pose_z',
            default_value='0.34',
            description='Camera 2 mount z offset in meters from base_link.',
        ),
        DeclareLaunchArgument(
            'camera_2_roll',
            default_value='0.0',
            description='Camera 2 roll in radians.',
        ),
        DeclareLaunchArgument(
            'camera_2_pitch',
            default_value=DEFAULT_CAMERA_UPWARD_PITCH_RAD,
            description='Camera 2 pitch in radians. Default is 40 degrees upward.',
        ),
        DeclareLaunchArgument(
            'camera_2_yaw',
            default_value='0.57',
            description='Camera 2 yaw in radians.绕z轴旋转0.57弧度（约32.7度）使相机向右偏转，以覆盖机器人前方更宽的区域。',
        ),
        DeclareLaunchArgument(
            'launch_imu',
            default_value='false',
            description='Launch the external WIT IMU driver.',
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/imu_usb',
            description='Serial port for the external IMU.',
        ),
        DeclareLaunchArgument(
            'imu_baud',
            default_value='115200',
            description='Baud rate for the external IMU.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used by robot_localization.',
        ),
    ]


def create_ascamera_driver_launch():
    ascamera_share = get_package_share_directory('ascamera')
    ascamera_launch_path = os.path.join(ascamera_share, 'launch', 'hp60c.launch.py')

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ascamera_launch_path),
        condition=IfCondition(LaunchConfiguration('launch_ascamera')),
        launch_arguments={
            'camera_1_usb_bus_no': LaunchConfiguration('camera_1_usb_bus_no'),
            'camera_1_usb_path': LaunchConfiguration('camera_1_usb_path'),
            'camera_2_enabled': LaunchConfiguration('camera_2_enabled'),
            'camera_2_usb_bus_no': LaunchConfiguration('camera_2_usb_bus_no'),
            'camera_2_usb_path': LaunchConfiguration('camera_2_usb_path'),
        }.items(),
    )


def create_ascamera_tf_nodes():
    camera_1_enabled = IfCondition(LaunchConfiguration('launch_ascamera'))
    camera_2_enabled = IfCondition(PythonExpression([
        "'", LaunchConfiguration('launch_ascamera'), "' == 'true' and '",
        LaunchConfiguration('camera_2_enabled'), "' == 'true'",
    ]))

    camera_1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ascamera_camera_1_tf',
        output='screen',
        condition=camera_1_enabled,
        arguments=[
            '--x', LaunchConfiguration('camera_1_pose_x'),
            '--y', LaunchConfiguration('camera_1_pose_y'),
            '--z', LaunchConfiguration('camera_1_pose_z'),
            '--roll', LaunchConfiguration('camera_1_roll'),
            '--pitch', LaunchConfiguration('camera_1_pitch'),
            '--yaw', LaunchConfiguration('camera_1_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', f'{ASCAMERA_CAMERA_1_NAMESPACE}_camera_link_0',
        ],
    )

    camera_2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ascamera_camera_2_tf',
        output='screen',
        condition=camera_2_enabled,
        arguments=[
            '--x', LaunchConfiguration('camera_2_pose_x'),
            '--y', LaunchConfiguration('camera_2_pose_y'),
            '--z', LaunchConfiguration('camera_2_pose_z'),
            '--roll', LaunchConfiguration('camera_2_roll'),
            '--pitch', LaunchConfiguration('camera_2_pitch'),
            '--yaw', LaunchConfiguration('camera_2_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', f'{ASCAMERA_CAMERA_2_NAMESPACE}_camera_link_0',
        ],
    )

    return [camera_1_tf, camera_2_tf]


def create_depth_scan_pipeline(scan_height):
    camera_1_enabled = IfCondition(LaunchConfiguration('launch_ascamera'))
    camera_2_enabled = IfCondition(PythonExpression([
        "'", LaunchConfiguration('launch_ascamera'), "' == 'true' and '",
        LaunchConfiguration('camera_2_enabled'), "' == 'true'",
    ]))

    scan_params = {
        'scan_time': 0.033,
        'range_min': 0.05,
        'range_max': 5.0,
        'scan_height': scan_height,
    }

    camera_1_depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_camera_1',
        output='screen',
        condition=camera_1_enabled,
        parameters=[{
            **scan_params,
            # Keep each scan in its own real camera mount frame instead of the
            # generic URDF camera_depth_frame.
            'output_frame': ASCAMERA_CAMERA_1_SCAN_FRAME,
        }],
        remappings=[
            ('depth', ASCAMERA_CAMERA_1_DEPTH_IMAGE),
            ('depth_camera_info', ASCAMERA_CAMERA_1_DEPTH_INFO),
            ('scan', ASCAMERA_CAMERA_1_SCAN_RAW),
        ],
    )

    camera_1_filter = Node(
        package='elderbot_bringup',
        executable='depth_scan_filter',
        name='depth_scan_filter_camera_1',
        output='screen',
        condition=camera_1_enabled,
        parameters=[{
            'input_topic': ASCAMERA_CAMERA_1_SCAN_RAW,
            'output_topic': ASCAMERA_CAMERA_1_SCAN,
        }],
    )

    camera_2_depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_camera_2',
        output='screen',
        condition=camera_2_enabled,
        parameters=[{
            **scan_params,
            'output_frame': ASCAMERA_CAMERA_2_SCAN_FRAME,
        }],
        remappings=[
            ('depth', ASCAMERA_CAMERA_2_DEPTH_IMAGE),
            ('depth_camera_info', ASCAMERA_CAMERA_2_DEPTH_INFO),
            ('scan', ASCAMERA_CAMERA_2_SCAN_RAW),
        ],
    )

    camera_2_filter = Node(
        package='elderbot_bringup',
        executable='depth_scan_filter',
        name='depth_scan_filter_camera_2',
        output='screen',
        condition=camera_2_enabled,
        parameters=[{
            'input_topic': ASCAMERA_CAMERA_2_SCAN_RAW,
            'output_topic': ASCAMERA_CAMERA_2_SCAN,
        }],
    )

    return [
        camera_1_depth_to_scan,
        camera_1_filter,
        camera_2_depth_to_scan,
        camera_2_filter,
    ]


def create_imu_node():
    return Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': ParameterValue(LaunchConfiguration('imu_baud'), value_type=int),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'frame_id': 'imu_link',
        }],
    )
