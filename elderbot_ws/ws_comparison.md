# elderbot_ws vs slam1_ws 完整对比

## 一、工作空间结构

| 对比项 | slam1_ws | elderbot_ws |
|--------|----------|-------------|
| 包数量 | 3个（slam_code, rplidar_ros, wit_ros2_imu） | 7个（elderbot, elderbot_base, elderbot_bringup, elderbot_description, elderbot_navigation, rplidar_ros, wit_ros2_imu） |
| 架构风格 | 所有代码塞进一个 slam_code 包 | 按功能分包（linorobot2 风格） |
| TF 发布方式 | static_tf.py 手写代码发布 | **URDF/Xacro 机器人描述文件** + robot_state_publisher |
| URDF | 无 | **有完整的 xacro 描述**（base, wheel, caster, sensors） |

## 二、导航算法对比（最核心的区别）

### 2.1 局部控制器（FollowPath）

| 参数 | slam1_ws (DWB) | elderbot_ws (RPP) |
|------|---------------|------------------|
| **插件** | `dwb_core::DWBLocalPlanner` | `RegulatedPurePursuitController` |
| **算法原理** | 动态窗口法：采样多条轨迹评分选最优 | 纯追踪法：追踪路径上的前方点 |
| 控制频率 | 35 Hz | 30 Hz |
| 最大线速度 | 2.0 m/s | **5.0 m/s** |
| 最大角速度 | 6.0 rad/s | **100.0 rad/s**（自转） |
| 线加速度 | 3.0 m/s² | 5.0 m/s² |
| 角加速度 | 5.0 rad/s² | **200.0 rad/s²** |
| 前方追踪距离 | 无（DWB 不用） | **1.0m**（lookahead_dist） |
| 自转对准 | 无 | **有**（rotate_to_heading，偏差>29°自动旋转） |
| 碰撞检测 | 有（BaseObstacle critic） | **关闭** |
| 速度调节 | 无 | **关闭**（use_regulated_linear_velocity_scaling=false） |
| DWB 评分器 | RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist | 无（RPP 不用 critics） |

### 2.2 全局规划器（GridBased）

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| **插件** | `NavfnPlanner`（Dijkstra/A*） | **`SmacPlanner2D`**（更优的搜索算法） |
| A* 搜索 | 启用 | 默认启用 |
| tolerance | 1.0 | **0.125**（更精确） |
| max_planning_time | 无限制 | **2.0s** |
| 路径平滑 | 无 | **有 smoother**（w_smooth=0.3, w_data=0.2） |
| max_iterations | 无限制 | **1000000** |

## 三、Costmap 代价地图对比

### 3.1 全局 Costmap

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| robot_radius | 0.25 | **0.20**（更准确） |
| inflation_radius | 0.35 | **0.40** |
| cost_scaling_factor | 4.5 | **0.5**（衰减更慢，更远离障碍物） |
| scan 话题 | /scan | **/scan_filtered** |
| obstacle_layer combination_method | 1（Maximum） | **0（Overwrite）**（能清除静态地图幽灵障碍物） |
| voxel_layer min_obstacle_height | 0.0 | **0.15**（过滤地面点云） |
| voxel_layer obstacle_min_range | 0.0 | **0.2**（过滤相机近场噪声） |

### 3.2 局部 Costmap

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| robot_radius | 0.25 | **0.20** |
| inflation_radius | 0.32 | **0.40** |
| cost_scaling_factor | 4.5 | **0.5** |
| scan 话题 | /scan | **/scan_filtered** |
| voxel_layer min_obstacle_height | 0.0 | **0.15** |
| voxel_layer obstacle_min_range | 0.0 | **0.2** |

## 四、Velocity Smoother 速度平滑器

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| max_velocity [x, y, θ] | [2.0, 0.0, 4.0] | **[5.0, 0.0, 200.0]** |
| max_accel [x, y, θ] | [2.0, 0.0, 4.0] | **[5.0, 0.0, 200.0]** |
| max_decel [x, y, θ] | [-2.0, 0.0, -4.0] | **[-5.0, 0.0, -200.0]** |
| feedback | OPEN_LOOP | OPEN_LOOP（不变） |

## 五、AMCL 定位

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| scan_topic | scan | **scan_filtered** |
| 其他参数 | 基本相同 | 来自 linorobot2 原版（基本相同） |

## 六、SLAM 建图

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| SLAM 工具 | SLAM Toolbox（同步模式） | SLAM Toolbox（**异步模式**） |
| scan_topic | /scan_filtered | /scan_filtered（相同） |
| 求解器 | CeresSolver | CeresSolver（不变） |
| 分辨率 | 0.05m | 0.05m（不变） |
| 激光范围 | 0.25-16.0m | 0.0-10.0m |

## 七、EKF 传感器融合

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| 频率 | 30 Hz | **50 Hz**（linorobot2 原版） |
| odom0 话题 | /odom | **odom/unfiltered** |
| odom0 融合 | vx, vyaw | vx, vy, vyaw（多了 vy） |
| imu0 话题 | /camera/gyro_accel/sample | /camera/gyro_accel/sample（相同） |
| imu0 融合 | vyaw | vyaw（相同） |

## 八、CAN 驱动 (can_driver.py)

| 参数 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| 硬件参数 | 完全相同（wheel 0.13m, separation 0.37m, ticks 5733） | 完全相同 |
| max_rpm_rate | **300.0** | **3000.0**（提速10倍） |
| 里程计话题 | /odom | **odom/unfiltered**（给 EKF 融合后发 /odom） |
| 定时器频率 | 20 Hz (0.05s) | 20 Hz (0.05s)（不变） |
| 关机处理 | 停止电机+解使能 | 停止电机+解使能（不变） |

## 九、Laser Filter 激光滤波

| 对比 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| 过滤方式 | **按角度过滤**（保留前方180°视野） | **按最小距离过滤**（去掉20cm内细杆点） |
| 是否在导航launch中启用 | **否**（只在SLAM时用） | **是**（导航和SLAM都启用） |
| 实现方式 | Python for 循环 | **numpy 向量化**（更快） |

## 十、Launch 文件

| 对比 | slam1_ws | elderbot_ws |
|------|----------|-------------|
| static_tf 节点 | **有**（手写 TF 发布） | **无**（用 URDF 替代） |
| initial_pose_pub 节点 | **有**（自动发初始位姿） | **无**（手动在 RViz 设） |
| laser_filter 节点 | 导航时**不启动** | 导航时**启动** |
| Nav2 延迟 | **无** | **3秒延迟**（等硬件初始化） |

## 十一、总结：关键差异

| 方面 | slam1_ws | elderbot_ws | 效果 |
|------|----------|-------------|------|
| 速度 | 最大 2 m/s | **最大 5 m/s** | 快 2.5 倍 |
| 自转 | 最大 6 rad/s | **最大 200 rad/s** | 几乎瞬间转向 |
| 控制器 | DWB（动态窗口，计算重） | **RPP**（纯追踪，极轻量） | 不卡顿 |
| 规划器 | NavfnPlanner（基础A*） | **SmacPlanner2D**（更优路径） | 路径更平滑 |
| 障碍物清除 | 不能清除旧障碍物 | **combination_method=0** | 搬走的障碍物能消失 |
| 地面假障碍物 | 没处理 | **min_obstacle_height=0.15** | 不会卡住 |
| 细杆干扰 | 角度过滤（且导航不启用） | **距离过滤（始终启用）** | 不受细杆干扰 |
| TF | 硬编码 Python | **URDF/Xacro** | 更规范、可维护 |
