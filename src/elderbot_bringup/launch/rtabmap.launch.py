import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home_dir = os.path.expanduser('~')

    elderbot_base_dir = get_package_share_directory('elderbot_base')
    ekf_params_file = os.path.join(elderbot_base_dir, 'config', 'ekf.yaml')

    camera_imu_topic = LaunchConfiguration('camera_imu_topic')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    fresh_db = LaunchConfiguration('fresh_db')
    localization = LaunchConfiguration('localization')
    initial_pose = LaunchConfiguration('initial_pose')
    approx_sync_max_interval = LaunchConfiguration('approx_sync_max_interval')
    neighbor_link_refining = LaunchConfiguration('neighbor_link_refining')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_description'), 'launch', 'description.launch.py']
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elderbot_navigation'), 'rviz', 'slam.rviz']
    )
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='深度相机发布的 IMU 话题'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(home_dir, 'maps', 'elderbot_rtabmap.db'),
            description='RTAB-Map 数据库文件路径'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='是否使用 SLAM 配置启动 RViz'
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='是否启动 RTAB-Map 图形界面'
        ),
        DeclareLaunchArgument(
            'fresh_db',
            default_value='true',
            description='启动时是否删除已有 RTAB-Map 数据库'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='是否以定位模式启动 RTAB-Map'
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='',
            description='定位模式下的初始位姿，格式为："x y z roll pitch yaw"'
        ),
        DeclareLaunchArgument(
            'approx_sync_max_interval',
            default_value='0.01',
            description='RTAB-Map 输入近似同步允许的最大时间差，单位为秒'
        ),
        DeclareLaunchArgument(
            'neighbor_link_refining',
            default_value='true',
            description='是否使用配准细化 RTAB-Map 相邻关键帧约束'
        ),

        Node(
            package='elderbot_bringup',
            executable='can_driver',
            name='can_driver',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
            ),
            launch_arguments={'serial_port': '/dev/rplidar'}.items()
        ),

        Node(
            package='elderbot_bringup',
            executable='laser_filter',
            name='laser_filter',
            output='screen',
            parameters=[{
                'range_min_filter': 0.20,
                'rear_blind_angle': 90.0,
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_camera_dir, 'launch', 'gemini_330_series.launch.py')
            ),
            launch_arguments={
                'enable_point_cloud': 'false',
                'depth_registration': 'true',
                'enable_colored_point_cloud': 'false',
                'enable_left_ir': 'false',
                'enable_right_ir': 'false',
                'time_domain': 'global',
                'log_level': 'info',
            }.items()
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'imu0': camera_imu_topic}],
            remappings=[('odometry/filtered', '/odom')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                # 机器人机体坐标系。RTAB-Map 会把它作为机器人本体的参考坐标，
                # 要和 URDF/TF 树中的底盘坐标系名称保持一致。
                'frame_id': 'base_footprint',
                # 外部里程计输入话题。这里接收 robot_localization 融合后的 /odom，
                # 为 RTAB-Map 提供连续的局部位姿估计。
                'odom_topic': '/odom',
                # 全局地图坐标系名称。RTAB-Map 会在该坐标系下维护地图，
                # 并根据建图/定位结果发布 map -> odom 的全局修正。
                'map_frame_id': 'map',
                # RTAB-Map 数据库路径。建图模式下保存地图、节点、约束和传感器数据；
                # 定位模式下从该文件加载已有地图。
                'database_path': database_path,
                # 是否启用定位模式。false 表示边走边建图；true 表示使用已有数据库定位，
                # 通常不再向地图中追加新的关键帧。
                'localization': localization,
                # 定位模式初始位姿。格式为 "x y z roll pitch yaw"，
                # 用于给全局定位一个初始搜索位置，留空时由 RTAB-Map 自行匹配。
                'initial_pose': initial_pose,
                # 是否订阅二维激光雷达 Scan 数据。启用后可用激光辅助建图、
                # 栅格地图生成和回环约束，提高平面环境稳定性。
                'subscribe_scan': 'true',
                # 激光雷达输入话题。这里使用 laser_filter 输出的 /scan_filtered，
                # 已经去除了近距离噪声和车体后方遮挡区域。
                'scan_topic': '/scan_filtered',
                # 是否直接订阅深度图输入。这里关闭直接深度订阅，
                # 由 rgbd_sync 节点统一同步 RGB、Depth 和 CameraInfo。
                'depth': 'false',
                # RGB 彩色图像话题。主要用于视觉特征提取、外观匹配和回环检测。
                'rgb_topic': '/camera/color/image_raw',
                # 深度图像话题。与 RGB 图像同步后形成 RGBD 数据，
                # 用于估计场景几何和辅助特征深度恢复。
                'depth_topic': '/camera/depth/image_raw',
                # 彩色相机内参话题。包含焦距、主点和畸变模型，
                # RTAB-Map 需要它把图像像素和深度值投影到三维空间。
                'camera_info_topic': '/camera/color/camera_info',
                # 是否启用 RGBD 同步。true 时会启动同步节点，
                # 将 RGB、Depth 和 CameraInfo 打包成 RTAB-Map 使用的 RGBD 消息。
                'rgbd_sync': 'true',
                # RGBD 同步是否采用近似时间同步。true 可容忍相机各话题时间戳轻微偏差，
                # 更适合真实传感器驱动下的非完全同步数据流。
                'approx_rgbd_sync': 'true',
                # RTAB-Map 多传感器输入是否采用近似同步。启用后 RGBD、Scan、Odom 等消息
                # 不必时间戳完全一致，能降低消息丢弃概率。
                'approx_sync': 'true',
                # 单个订阅话题的缓存队列长度。数值越大越能吸收短时间消息堆积，
                # 但也会增加内存占用和潜在延迟。
                'topic_queue_size': '120',
                # 同步器内部队列长度。影响近似同步在多个话题之间寻找匹配消息的范围，
                # 过小可能同步失败，过大可能增加处理延迟。
                'sync_queue_size': '100',
                # ROS 2 QoS 档位。rtabmap_launch 中 1=Reliable, 2=Best Effort。
                'qos': '1',
                # 近似同步允许的最大时间差，单位为秒。0.03 表示最多接受 30 ms 内的消息匹配，
                # 需要结合相机、雷达和里程计发布时间戳质量调整。
                'approx_sync_max_interval': approx_sync_max_interval,
                # 是否启动 RTAB-Map 内置视觉里程计。这里关闭，因为位姿来源由底盘里程计、
                # IMU 和 robot_localization 融合后的 /odom 提供。
                'visual_odometry': 'false',
                # 是否启动 RTAB-Map 内置 ICP 里程计。这里关闭，避免与外部 /odom 重复发布
                # 或产生两套里程计估计。
                'icp_odometry': 'false',
                # 是否发布 map -> odom 的 TF。建图/定位结果需要通过该变换修正全局漂移，
                # 因此这里开启。
                'publish_tf_map': 'true',
                # 是否发布 odom -> base 的 TF。这里关闭，因为该变换应由底盘里程计
                # 或 robot_localization 负责，避免 TF 发布源冲突。
                'publish_tf_odom': 'false',
                # 等待 TF 可用的最长时间，单位为秒。传感器消息到达时，
                # RTAB-Map 会在该时间内等待所需坐标变换。
                'wait_for_transform': '0.5',
                # 是否启动 RTAB-Map 自带的可视化界面。由 launch 参数 rtabmap_viz 控制，
                # 便于按需查看图优化、回环和数据库状态。
                'rtabmap_viz': rtabmap_viz,
                # 是否让 rtabmap_launch.py 自己启动 RViz。这里固定关闭，
                # 改由本文件末尾的 RViz 节点和 slam.rviz 配置统一控制。
                'rviz': 'false',
                # 额外传给 RTAB-Map 节点的原生命令行参数。
                # 这里集中放置图优化、栅格地图和数据库启动行为等细粒度配置。
                'rtabmap_args': [
                    # Reg/Strategy：选择配准策略。1 表示使用 ICP 配准，
                    # 适合结合二维激光雷达约束平面移动机器人的位姿优化。
                    '--Reg/Strategy 1 '
                    # Reg/Force3DoF：强制使用 x、y、yaw 的二维位姿优化，
                    # 忽略 z、roll、pitch 漂移，适合室内平面移动机器人。
                    '--Reg/Force3DoF true '
                    # RGBD/NeighborLinkRefining：用配准细化相邻关键帧约束。
                    # 二维激光建图时默认开启，可减少轮速里程计转弯误差造成的墙角重影。
                    '--RGBD/NeighborLinkRefining ', neighbor_link_refining, ' ',
                    # RGBD/ProximityBySpace：在空间上接近的历史节点之间做局部回环检测。
                    # 这可以让经过同一墙角时更早闭合局部约束，降低多边重影。
                    '--RGBD/ProximityBySpace true '
                    # RGBD/ProximityByTime：关闭短时记忆里的时间邻近检测，避免刚走过的相似局部结构重复加约束。
                    '--RGBD/ProximityByTime false '
                    # RGBD/ProximityPathMaxNeighbors：允许合并邻近路径上的多个 scan 做一对多匹配，
                    # 对墙面、走廊这类几何重复场景更稳。
                    '--RGBD/ProximityPathMaxNeighbors 10 '
                    # RGBD/OptimizeMaxError：拒绝会造成过大图优化误差的回环/邻近约束。
                    '--RGBD/OptimizeMaxError 3 '
                    # Mem/STMSize：增大短时记忆，减少刚经过区域被过早重复建成多个局部节点。
                    '--Mem/STMSize 30 '
                    # Mem/NotLinkedNodesKept：丢弃没有成功建立约束的节点，减少坏 scan 留在地图中。
                    '--Mem/NotLinkedNodesKept false '
                    # Icp/*：二维激光点到点匹配参数。阈值偏保守，宁愿不接受坏约束，也不要把墙角拉成多边。
                    '--Icp/PointToPlane false '
                    '--Icp/VoxelSize 0.05 '
                    '--Icp/MaxCorrespondenceDistance 0.15 '
                    '--Icp/CorrespondenceRatio 0.20 '
                    '--Icp/MaxTranslation 0.30 '
                    '--Icp/MaxRotation 0.35 '
                    '--Icp/RangeMin 0.20 '
                    '--Icp/RangeMax 8.0 '
                    # Optimizer/GravitySigma：已强制 3DoF，关闭重力约束，避免 IMU 姿态约束影响平面图优化。
                    '--Optimizer/GravitySigma 0 '
                    # Grid/Sensor：选择占据栅格地图的数据来源。0 表示优先使用激光雷达 Scan，
                    # 相比仅用深度相机，二维导航地图通常更干净稳定。
                    '--Grid/Sensor 0 '
                    # Grid/3D：只生成二维占据栅格，避免 3D 投影/OctoMap 相关开销影响实时性。
                    '--Grid/3D false '
                    # Grid/RangeMin：生成栅格地图时忽略小于该距离的测距点，单位为米，
                    # 用于过滤车体附近、雷达盲区或深度噪声造成的障碍。
                    '--Grid/RangeMin 0.20 '
                    # Grid/RangeMax：限制过远、较易抖动的激光点进入占据栅格。
                    '--Grid/RangeMax 8.0 '
                    # Grid/RayTracing：启用射线追踪清除自由空间。
                    # 雷达从原点到命中点之间的区域会被标记为可通行。
                    '--Grid/RayTracing true',
                    # delete_db_on_start：当 fresh_db 为 true 时追加该启动参数，
                    # 启动前删除旧数据库，确保本次运行从空地图重新开始。
                    PythonExpression([
                        "' --delete_db_on_start' if '",
                        fresh_db,
                        "' == 'true' else ''",
                    ]),
                ],
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(rviz)
        ),
    ])
