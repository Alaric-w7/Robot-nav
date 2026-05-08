# Patrol Action 使用说明

## 1. 功能概览

`Patrol.action` 用来触发 ElderBot 的自动巡航。  
`patrol_node.py` 现在是一个 action server，启动后会监听：

- action 名称：`/patrol`
- action 类型：`elderbot_navigation/action/Patrol`

巡航点默认仍然来自：

- `~/elderbot_ws/src/elderbot_navigation/config/patrol_waypoints.yaml`

## 2. 先编译

```bash
cd ~/elderbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elderbot_navigation
source ~/elderbot_ws/install/setup.bash
```

## 3. 启动 action server

先启动导航栈，再单独启动巡航 action server：

```bash
cd ~/elderbot_ws
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash

ros2 run elderbot_navigation patrol_node.py \
  --ros-args \
  --params-file ~/elderbot_ws/src/elderbot_navigation/config/patrol_waypoints.yaml
```

如果你想继续使用原来的脚本入口，也可以直接执行：

```bash
cd ~/elderbot_ws
bash run_patrol.sh
```

这个脚本现在会在后台启动 `patrol_node.py`，然后通过 action 发送一个“无限循环巡航”的 goal。

## 4. 如何调用

### 4.1 使用默认配置，执行 1 轮巡航

```bash
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash

ros2 action send_goal --feedback /patrol elderbot_navigation/action/Patrol "{}"
```

说明：

- `waypoint_names` 留空：使用 YAML 中配置的全部巡逻点
- `repeat_count` 默认是 `0`：在 `loop_forever=false` 时，等价于执行 1 轮
- 停留时间和导航超时使用 `patrol_node.py` 当前参数值

### 4.2 无限循环巡航

```bash
ros2 action send_goal --feedback /patrol elderbot_navigation/action/Patrol \
  "{loop_forever: true}"
```

### 4.3 只巡航指定点位

```bash
ros2 action send_goal --feedback /patrol elderbot_navigation/action/Patrol \
  "{waypoint_names: ['point_a', 'point_c'], repeat_count: 2}"
```

上面的命令表示：

- 仅巡航 `point_a` 和 `point_c`
- 连续执行 2 轮

### 4.4 覆盖默认停留时间和导航超时

```bash
ros2 action send_goal --feedback /patrol elderbot_navigation/action/Patrol \
  "{repeat_count: 1, override_wait_duration: true, wait_duration: 5.0, override_navigation_timeout: true, navigation_timeout: 90.0}"
```

## 5. Goal 字段说明

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `waypoint_names` | `string[]` | 要巡航的点名列表；为空时使用配置文件中的全部点 |
| `repeat_count` | `int32` | 巡航轮数；当 `loop_forever=false` 且该值 `<=0` 时，默认跑 1 轮 |
| `loop_forever` | `bool` | 是否无限循环巡航 |
| `override_wait_duration` | `bool` | 是否使用 goal 里的 `wait_duration` 覆盖默认停留时间 |
| `wait_duration` | `float32` | 每个点到达后的停留秒数 |
| `override_navigation_timeout` | `bool` | 是否使用 goal 里的 `navigation_timeout` 覆盖默认导航超时 |
| `navigation_timeout` | `float32` | 单个点的导航超时时间（秒） |

## 6. Feedback / Result

发送 goal 时加上 `--feedback`，可以看到类似这些反馈：

- `current_round`：当前第几轮
- `current_waypoint_index`：当前第几个巡逻点
- `current_waypoint_name`：当前巡逻点名称
- `state`：当前状态，可能是 `navigating`、`arrived`、`waiting`、`timed_out`、`failed`
- `elapsed_time`：当前状态已持续的时间

任务结束后，result 会返回：

- `success`
- `rounds_completed`
- `waypoints_succeeded`
- `waypoints_failed`
- `waypoints_timed_out`
- `message`

## 7. 查看 action 定义

```bash
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash
ros2 interface show elderbot_navigation/action/Patrol
```
