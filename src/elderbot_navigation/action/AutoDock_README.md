# AutoDock Action 使用说明

## 1. 功能概览

`AutoDock.action` 用来触发 ElderBot 自动回充。

`auto_dock_node.py` 现在同时具备两种能力：

- 后台监控模式：电量低于 `battery_threshold` 时自动回充
- Action 模式：收到 `/auto_dock` goal 后，立即强制回充，不受电量阈值限制

也就是说，别人只要调用 action，就算当前电量高于 20%，小车也会执行：

1. 读取 `dock_pose.yaml`
2. 导航到回充准备点
3. 去激活 Nav2 controller
4. 倒车寻找充电电流
5. 恢复 Nav2

Action 信息：

- action 名称：`/auto_dock`
- action 类型：`elderbot_navigation/action/AutoDock`

## 2. 先编译

```bash
cd ~/elderbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elderbot_navigation
source ~/elderbot_ws/install/setup.bash
```

## 3. 启动节点

最推荐的启动方式是继续使用现有脚本：

```bash
bash ~/elderbot_ws/run_autodock.sh
```

这个脚本现在会同时做两件事：

- 启动自动回充监控
- 启动 `/auto_dock` action server

如果你希望继续沿用原来的“几乎立刻触发低电量回充”方式，这个命令仍然可用：

```bash
bash ~/elderbot_ws/run_autodock.sh --ros-args -p battery_threshold:=0.99
```

这条命令的兼容性已经保留好了。

## 4. 如何调用 action

### 4.1 最简单的调用方式

```bash
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash

ros2 action send_goal --feedback /auto_dock elderbot_navigation/action/AutoDock "{}"
```

上面的 goal 一发出去，就会强制执行回充流程，不看当前电量是否低于阈值。

### 4.2 带来源标记的调用方式

```bash
ros2 action send_goal --feedback /auto_dock elderbot_navigation/action/AutoDock \
  "{request_source: 'remote_operator'}"
```

`request_source` 只是写进日志里，方便排查是谁触发的。

## 5. Goal / Feedback / Result 字段

### 5.1 Goal

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `request_source` | `string` | 请求来源描述，可留空，只用于日志记录 |

### 5.2 Feedback

常见 `stage` 状态包括：

- `waiting_nav2`
- `navigating_to_prep`
- `stabilizing`
- `deactivating_nav2`
- `backing_up`
- `finished`
- `already_charging`
- `error`

反馈字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `stage` | `string` | 当前阶段 |
| `is_charging` | `bool` | 当前是否检测到充电 |
| `battery_percent` | `float32` | 当前电量百分比，范围 0 到 100 |
| `charging_current` | `float32` | 当前充电电流 |
| `message` | `string` | 当前阶段说明 |

### 5.3 Result

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `success` | `bool` | 回充流程是否成功完成 |
| `charged` | `bool` | 是否最终检测到充电电流 |
| `battery_percent` | `float32` | 结束时电量百分比 |
| `charging_current` | `float32` | 结束时电流 |
| `message` | `string` | 结果说明 |

## 6. 运行行为说明

- 如果机器人当前已经在充电，action 会直接返回成功，不会重复执行导航和倒车。
- 如果已经有一个回充流程在执行中，新的 `/auto_dock` goal 会被拒绝。
- Action 触发的回充是“强制回充”，不检查 `battery_threshold`。
- 后台监控逻辑仍然保留，所以这个节点既能自动回充，也能被外部显式调用。

## 7. 查看 action 定义

```bash
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash
ros2 interface show elderbot_navigation/action/AutoDock
```
