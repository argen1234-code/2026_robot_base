# 识别人并靠近 / 连续跟随（2026-11-06）

本工程的人体闭环复用现有 Nav2，不向底盘直接发速度命令：

```text
YOLO 人像检测（彩色 bbox）──┐
                            ├─ 相机 bbox 与 2D 雷达 /scan_filtered 聚类关联
2D 雷达（唯一测距来源）─────┘        ↓
                          map 中的人位置（不用深度相机测距）
                                    ↓
多人时选画面中心最近者（近中心并列则拒绝，避免换人）
                                    ↓
在人的前方 standoff_m 处生成虚拟目标点 → Nav2 规划 → 行驶
                                    ↓
到达 → 「靠近一次」结束；「连续跟随」等待同一人移动 > follow_replan_m 后重新规划
```

> 距离判定只使用 2D 雷达。Orbbec 深度相机可以被启动，但其 `depth_m`
> 不参与视觉决策层的定位/停车判定；停车点由 bbox 与雷达聚类关联得到的人在
> `map` 中的位置计算。彩色框只负责“这是不是人、框在画面哪里”。

语音唤醒后的完整动作：

```text
唤醒词 → ASR → 语音决策层确认“来找我”
  → 视觉决策层进入 acquiring
  → 1 s 内没有目标则调用 Nav2 /spin 缓慢旋转 360°
  → 检测到人框 + 雷达关联成功 → 停止旋转
  → 连续 3 帧/0.4 s 稳定 → 生成人的前方 standoff 虚拟点
  → ComputePathToPose + NavigateToPose（复用先验地图导航）
  → 行驶中持续用最新人位置重新规划；到达后跟随模式等待目标再次移动
```

## 与参考工程的关系

参考 `Wenyu-code/wastebasket_car` 的 `maixcam/follow_person.py`：

- 多人时选水平方向离画面中心最近的人；
- 无人立即停车，离人太近停车，否则左转/右转/前进；
- 网页端有智能模式开关和人物状态回显；
- 命令限频、发送失败降级。

本工程保留其目标选择与“到达后继续跟随”的闭环思路，但不用像素面积/HTTP 直接控车，
而是用雷达聚类测距、代价地图检查和 Nav2 避障；到达判定用 `standoff_m` 与
`arrival_tolerance_m`，连续跟随用同一目标的 map 位置位移触发重规划。新增的安全边界：

- 近中心多人无法区分时直接拒绝（`ambiguous_person_choice`），不静默换人；
- 已有目标的重捕获窗口内不会切换到别人；
- 目标短暂丢失保留位置，超过 `reacquire_timeout_s` 结束任务；
- 连续跟随总时长受 `follow_max_s` 限制；
- 手动 Foxglove 目标、`/stop_request`、导航超时都会立即释放人体任务。

## 启动

统一入口会自动启动导航、视觉和分层决策层，不再需要单独启动找人节点。

```bash
cd ~/2026_robot_base

# 全栈预览：不 ARM，只显示候选目标和路线
bash scripts/robot.sh start

# 实车连续跟随（需要 --motor，且 calibration_confirmed=true）
bash scripts/robot.sh start --motor --task follow

# 精简跟随：不启动 AI，只保留 Astra 视觉
bash scripts/robot.sh start --no-ai --vision astra --motor --task follow
```

不传 `--task follow` 时，决策层空闲；如果启动时满足
`--motor` + 标定确认，也可以在网页点「连续跟随」启动任务。
`--foxglove` 可额外打开 8765 可视化；默认关闭以节省 CPU。

**导航模式必须是 `--nav localize`（默认）**：加载 `~/maps` 下最新的
`.posegraph` 先验地图，和已经调通的导航链路完全一致；不要用
`--nav mapping`，人体任务在 mapping 模式下不会启动。

语音入口：

```text
唤醒词“你好小亚” → “来找我”/“过来” → 语音确认“好的”
  → 决策层把 approach_user 映射为 follow（连续追踪）
  → 视觉层开始 360° 缓慢找人
```

`approach_user` 不再冻结第一个停车点，而是按“连续跟随”处理：行驶中
目标移动超过 `target_move_cancel_m` 会取消当前 Nav2 目标并重新规划；
到达停车距离后保持任务，目标再移动超过 `follow_replan_m` 会重新靠近。

网页：`http://<树莓派IP>:8878`。页面按钮：

- **靠近一次**：行驶过程中同样追踪人，到达停车距离后任务结束；
- **连续跟随**：语音 `approach_user` 也走这一模式；到达后保持任务，
  目标移动超过 `follow_replan_m` 自动重新靠近；
- **停止靠近任务**：取消当前 Nav2 目标并结束任务。

状态接口：`GET /status`，包含 `state`、`target_px`、`target_offset_px`、
`distance_m`、`follow`、`follow_wait`、`near_person` 等字段。
调试数据：`GET /debug`。

## 实车测试：直接开始找人追踪

前置条件：

- `~/maps` 下已有可用的 `.posegraph`，`robot.sh` 默认 `--nav localize` 加载先验地图；
- `/dev/ydlidar`、`/dev/stm32`、`/dev/astra-color` 存在；
- `config/person_approach.json` 中 `calibration_confirmed: true`；
- 场地空旷，急停按钮可用；人站在车前方 1.5–3 m 处。

最快路径（启动即进入找人/追踪，不需要说话）：

```bash
cd ~/2026_robot_base
bash scripts/robot.sh start --motor --task follow \
  --vision astra --no-depth --map ~/maps/maphotel
```

把 `~/maps/maphotel` 换成你实际测试区域的 posegraph 前缀即可；不写 `--map`
则使用 `~/maps` 下最新的 `.posegraph`。`--vision astra` 避免启动手部相机，
`--no-depth` 跳过 Orbbec 深度（距离判定只用 2D 雷达）。

等价的分步路径：

```bash
bash scripts/robot.sh start --motor --vision astra --no-depth --map ~/maps/maphotel
# 等待终端打印 [robot] READY，浏览器打开 http://<树莓派IP>:8878，点「连续跟随」
```

语音路径使用同一条启动命令（去掉 `--task follow`），然后：

```text
“你好小亚” → “来找我”/“过来” → 听到“需要我去到你身边吗” → 再次唤醒 + “好的”
```

注意：当前 ASR 节点是“一次唤醒只录一句”，确认“好的”时需要再次说唤醒词
（“你好小亚” → “好的”）。如果只想验证视觉找人/追踪，直接使用上面的
`--task follow` 或网页「连续跟随」，不依赖语音确认。

预期状态（`/person_nav/status` 的 `state` 字段）：

```text
acquiring → searching_person（无人时缓慢转 360°）
          → stabilizing（找到人，连续确认）
          → planning / ready（生成人的前方 standoff 虚拟点）
          → navigating（Nav2 行驶，画面居中叠加生效）
          → arrived_waiting_for_target_move（连续跟随到达）
          → 人移动 > 0.6 m → 再次 planning/navigating
```

观察与停止：

```bash
# 状态、目标像素偏差、距离、是否在等目标移动
ROS_DOMAIN_ID=1 ros2 topic echo /person_nav/status
# 发给 Nav2 的虚拟停车点（人的前方 standoff_m）与可视化标记
ROS_DOMAIN_ID=1 ros2 topic echo /person_nav/goal
ROS_DOMAIN_ID=1 ros2 topic echo /person_nav/markers
# 视觉居中包（中心偏差、是否 active）
ROS_DOMAIN_ID=1 ros2 topic echo /person_nav/centering_target
# 检测框与雷达关联调试
curl -s http://<树莓派IP>:8878/debug | head -c 2000

# 网页点「停止靠近任务」，或：
ROS_DOMAIN_ID=1 ros2 topic pub --once /stop_request std_msgs/msg/Bool "{data: true}"
# 或直接停整栈
bash scripts/robot.sh stop
```

若机器人不转圈：先确认 `/spin` action 有服务端（`ros2 action list | grep spin`），
并查看 `~/rosbags/robot/latest/decision.log` 中是否有 `waiting_for_spin`。
若找到人但不动：确认 `execute=true`、`--motor`、先验地图定位正常，以及
`/navigate_to_pose` action 可用。

## 参数（`config/person_approach.json`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `standoff_m` | 0.5 | 车头到人的停车距离（米） |
| `robot_front_m` | 0.246 | 车头到 `base_footprint` 的距离 |
| `arrival_tolerance_m` | 0.02 | 到达判定容差 |
| `search_delay_s` | 1.0 | 任务启动后多久开始转圈找人 |
| `search_spin_target_yaw_rad` | 2π | 每次找人的旋转角度（约 360°） |
| `search_spin_time_allowance_s` | 30 | Nav2 `/spin` 超时；先验图参数约 0.55 rad/s，360° 约 12 s，30 s 足够 |
| `search_timeout_s` | 120 | 找人总时长，超时结束 |
| `target_move_cancel_m` | 0.65 | 导航中同一目标位移超过该值，取消当前目标并重规划 |
| `follow_replan_m` | 0.6 | 连续跟随：到达后目标移动超过该距离才重新规划 |
| `follow_max_s` | 300 | 连续跟随总时长上限；0 表示不限制 |
| `multi_person_center_margin_px` | 32 | 多人时第一、第二目标的中心偏差小于该值则拒绝选择（按 640 宽换算） |
| `occlusion_grace_s` | 0.8 | 短暂漏检保留目标的时长 |
| `reacquire_timeout_s` | 8 | 超过该时长未找回同一目标则结束 |
| `near_person_bottom_fraction` | 0.98 | bbox 底边超过画面高度的比例，视为“人已到跟前” |
| `near_person_width_fraction` | 0.42 | bbox 宽度超过画面宽度的比例，视为“人已到跟前” |

## 状态

| state | 含义 |
|---|---|
| `acquiring` | 人体任务已启动，等待目标 |
| `searching_person` | 正在通过 Nav2 `/spin` 缓慢旋转 360° 找人 |
| `stabilizing` | 已检测到人，等待同一目标连续稳定 |
| `planning` / `ready` | 正在调用 Nav2 规划 / 虚拟停车点就绪 |
| `navigating` | 正在靠近；跟随模式下目标移动会触发重规划 |
| `arrived` | 已到达（单次任务结束） |
| `arrived_waiting_for_target_move` | 连续跟随：到达并等待同一目标移动 |
| `near_person_distance_unreliable` | 人已到跟前、脚部出画，雷达近距离关联不可用；按“人在”处理，不取消任务 |
| `ambiguous_person_choice` | 多人都在画面中心附近，拒绝静默选择 |
| `paused_waiting_for_target` | 导航中目标暂时丢失，已暂停并等待重捕获 |
| `target_reacquire_timeout` | 长时间未找回目标，任务结束 |
| `follow_timeout` | 连续跟随达到总时长上限 |

## 测试

```bash
cd ~/2026_robot_base
# 纯几何/居中（不包含 ROS 集成测试）
PYTHONPATH=$PWD/robot_ros2_ws/src/robot_base_decision \
  python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
  -p 'test_person_approach.py' -v
PYTHONPATH=$PWD/robot_ros2_ws/src/robot_base_decision \
  python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
  -p 'test_person_centering.py' -v

# ROS 集成（domain 79，假 Nav2/雷达/TF，不动真实电机）
micromamba run -n ros_jazzy bash -c \
  'source robot_ros2_ws/install/setup.bash && \
   PYTHONPATH="$PWD/robot_ros2_ws/src/robot_base_decision:$PYTHONPATH" \
   python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
   -p "test_person_approach_ros.py" -v'
```

ROS 集成测试覆盖：单次靠近到达、连续跟随重靠近、导航中目标移动重规划、
遮挡暂停/恢复、超时取消、多人歧义拒绝、手动目标抢占、居中包发布、
`approach_user` 映射为 follow。
