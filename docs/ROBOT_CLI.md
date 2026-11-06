# 统一机器人启动入口

唯一入口：

```bash
scripts/robot.sh start [options]
scripts/robot.sh stop [--force]
scripts/robot.sh status
```

旧的 `start_host_stack.sh`、`start_all.sh`、`start_stack.sh`、
`start_person_approach.sh`、`stop_all.sh` 已删除；底层 ROS launch 和调试工具
（`tools/run_nav2_replay.sh` 等）保留。

## 默认行为

不带参数时启动全栈：

```bash
scripts/robot.sh start
```

- 导航：重定位模式，使用 `~/maps` 最新 posegraph
- AI 语音：整栈启动
- 视觉：Astra（彩色/深度/人像）+ 手部相机
- 决策层：最上层 + 导航/语音/视觉决策层全开
- 电机：不 ARM
- Foxglove：关闭

启动完成后打印 READY、找人网页和日志目录。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--nav mapping\|localize\|none` | `localize` | 导航栈；`none` 表示不启动导航 |
| `--map new\|latest\|<prefix>` | mapping=`new`；localize=`latest` | 建图新建；`latest` 或 posegraph 前缀续建/定位 |
| `--map-start-pose "[x,y,yaw]"` | `[0.0,0.0,0.0]` | 指定初始位姿（重定位） |
| `--map-at-dock` | off | 使用建图时的停靠位姿 |
| `--ai` / `--no-ai` | on | AI 语音整栈 |
| `--vision astra\|hand\|both\|none` | `both` | Astra 与手部相机可独立选择 |
| `--foxglove` | off | 启动 Foxglove 8765 |
| `--motor` | off | ARM 底盘；需要 `/dev/stm32` |
| `--decision` / `--no-decision` | 非建图=on；建图=off | 决策层总开关 |
| `--no-nav-decision` | off | 关闭导航栈决策层 |
| `--no-voice-decision` | off | 关闭语音栈决策层 |
| `--no-vision-decision` | off | 关闭视觉栈决策层 |
| `--task none\|follow\|preview` | `none` | 决策层任务可选启动 |
| `--restart` | off | 先停止已有栈 |
| `--log-dir <dir>` | `~/rosbags/robot/<timestamp>` | 日志目录 |

## 常用组合

```bash
# 建图：默认新建地图
scripts/robot.sh start --nav mapping --map new --ai --vision both --foxglove

# 建图：续建指定 posegraph
scripts/robot.sh start --nav mapping --map /home/pi/maps/robot_map_20261105_054354

# 重定位：最新地图，只导航 + 导航决策层
scripts/robot.sh start --nav localize --map latest --no-ai --vision none

# 重定位：指定地图，只定位，不要导航决策层
scripts/robot.sh start --nav localize --map /home/pi/maps/robot_map_20261105_054354 \
  --no-ai --vision none --no-nav-decision

# 只启动 AI 语音
scripts/robot.sh start --nav none --ai --vision none

# 只启动 Astra 视觉（不启动 AI）
scripts/robot.sh start --nav none --no-ai --vision astra

# 只启动手部相机
scripts/robot.sh start --nav none --no-ai --vision hand

# 全栈 + Foxglove + 电机 + 找人跟随
scripts/robot.sh start --foxglove --motor --task follow

# 全栈但不要任何决策层
scripts/robot.sh start --no-decision

# 查看/停止
scripts/robot.sh status
scripts/robot.sh stop
```

## 安全校验

- 建图模式不接受 `--decision` / `--task`；只允许 Foxglove 手动设点建图。
- `--task follow` 必须同时满足：`--motor`、`config/person_approach.json`
  的 `calibration_confirmed=true`、`--nav localize`、`--vision astra`。
- `--task preview` 不需要电机，但需要导航重定位和 Astra 视觉。
- `--motor` 需要导航栈和 `/dev/stm32`。
- `--foxglove` 需要导航栈。
- 已有栈运行时拒绝启动；使用 `--restart` 或先 `stop`。

## 视觉独立性

- 选择 Astra 时，`astra_color_publisher` 是 `/dev/astra-color` 的唯一 owner，
  发布 `/camera/color/image_raw/compressed`（640×480、10 Hz、JPEG）。
- AI 同时启动时使用 `camera_source=ros_compressed`，从该话题取图，不重复打开相机。
- 不选 Astra、只启动 AI 时，AI 的 `model_service` 使用 `camera_source=uvc`。
- 手部相机走 `/dev/hand-cam` + MediaPipe + UDP + `hand_pose_bridge`，独立于 AI。

## 日志与状态

- 日志：`$HOME/rosbags/robot/<时间戳>/`，`latest` 指向最新目录。
- 组件日志：`nav.log`、`ai.log`、`color.log`、`depth.log`、`person.log`、
  `hand_det.log`、`hand_bridge.log`、`decision.log`。
- 清单：`$HOME/.robot/manifest`（组件名、PGID、PID）。
- `scripts/robot.sh status` 显示组件状态和 8765/8878 端口。
- `stop` 先发 SIGINT（导航触发 DISARM），等待 15 s，再 TERM/KILL。
  `stop --force` 缩短等待，可能跳过正常 DISARM。

## 性能提示

- 全栈（AI + 手部 + 人像 + 决策层）在 Pi 5 上 CPU 很紧；人像推理约
  0.5–0.9 s。需要流畅跟随时优先用精简配置：
  ```bash
  scripts/robot.sh start --no-ai --vision astra --nav localize --motor --task follow
  ```
  或关闭手部相机：`--vision astra`。
- Foxglove 客户端会额外占用 CPU；调试找人优先用 `http://<pi>:8878`。
- 建图模式决策层关闭；需要通过 Foxglove 手动设点。

## 测试

```bash
# 纯函数/ROS 集成测试
cd robot_ros2_ws
micromamba run -n ros_jazzy bash -c '
  source "$CONDA_PREFIX/setup.bash"; source install/setup.bash
  cd src/robot_base_decision
  python3 test/test_person_approach.py
  python3 test/test_person_centering.py
  python3 test/test_person_approach_ros.py
'
```
