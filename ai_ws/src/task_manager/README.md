# task_manager

目标搜索、接近与速度仲裁。

## 状态机

```text
IDLE -> SEARCHING -> APPROACHING -> ARRIVED
                  -> SEARCH_TIMEOUT
        any       -> STOPPED
```

- `SEARCHING`：以 `0.35 rad/s` 原地旋转，`/perception/target_debug` 连续 3 帧
  `valid:true` 后进入接近；
- `APPROACHING`：用 `/perception/target_pose`（相机光学坐标系，单位 m）
  做转向 + 前进；
- `ARRIVED`：到达停靠距离，零速；
- `TARGET_LOST`：接近过程中目标位姿超过 0.6 s 未更新，零速；
- `STOPPED`：收到 `/stop_request` 或 `/task_command` 的 stop。

## 接近控制

```text
bearing = atan2(x, z)
distance = sqrt(x^2 + y^2 + z^2)

|bearing| > 0.08 rad : 原地转向，线速度 0
否则                : v = clamp(0.6 * (distance - stop_distance), 0, 0.15)
                      w = clamp(1.0 * bearing, -0.4, 0.4)
```

停靠距离按以下参数计算：

```text
horizontal_camera = base_standoff - camera_forward_offset
                = 1.0 - 0.12 = 0.88 m
height_diff     = target_height - camera_height
                = 1.2 - 0.5 = 0.7 m
stop_distance_camera = hypot(0.88, 0.7) = 1.124 m
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/target_debug` | `String` | 视觉节点输出，JSON |
| `/perception/target_pose` | `PoseStamped` | 目标中心，相机光学坐标系 |
| `/task_command` | `String` | `search` / `stop` |
| `/summon_request` | `String` | 语音快速意图“过来” |
| `/stop_request` | `Bool` | 停止 |
| `/cmd_vel_search` | `Twist` | 搜索/接近速度 |
| `/task_mode` | `String` | `search` / `idle` |
| `/task_state` | `String` | 状态 |
| `/cmd_vel` | `Twist` | 仲裁后的唯一速度出口 |

## 运行

```bash
docker exec -it ros_humble_ai_dev bash
source /opt/ros/humble/setup.bash
source /root/yahboom_ws/install/setup.bash
export ROS_DOMAIN_ID=42

bash /root/yahboom_ws/src/perception/scripts/setup_orbbec_uvc.sh
ros2 launch task_manager task_manager.launch.py dry_run:=true
```

```bash
ros2 topic pub --once /task_command std_msgs/msg/String "{data: 'search'}"
ros2 topic echo /task_state
ros2 topic pub --once /stop_request std_msgs/msg/Bool "{data: true}"
```

实车测试：

```bash
# 轮子架空，确认方向正确后再落地
ros2 launch task_manager task_manager.launch.py dry_run:=false
```

## 安全注意

- `dry_run:=true` 时 `velocity_mux` 始终输出零 `/cmd_vel`；
- `dry_run:=false` 时底盘会真实运动，必须有人看管；
- 不要同时运行旧的 `motion_gate`，两者都会发布 `/cmd_vel`；
- `lidar_safety: true` 默认开启；接上 `/scan` 后会做前向近障停车；
- 当前接近假设相机**没有左右偏航安装角**；如果相机有 yaw 偏置，需要加偏置补偿。
