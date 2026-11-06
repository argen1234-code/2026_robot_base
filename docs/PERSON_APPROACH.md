# 识别人并靠近 / 连续跟随（2026-11-06）

本工程的人体闭环复用现有 Nav2，不向底盘直接发速度命令：

```text
YOLO 人像检测 ──┐
                ├─ 相机 bbox + 雷达聚类关联 → map 中的人位置
Orbbec 深度 ────┘
        ↓
多人时选画面中心最近者（近中心并列则拒绝，避免换人）
        ↓
站在人的前方 standoff_m 处生成目标 → Nav2 规划 → 行驶
        ↓
到达 → 「靠近一次」结束；「连续跟随」等待同一人移动 > follow_replan_m 后重新规划
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

网页：`http://<树莓派IP>:8878`。页面按钮：

- **靠近一次**：到达停车距离后任务结束；
- **连续跟随**：到达后保持任务，目标移动超过 `follow_replan_m` 自动重新靠近；
- **停止靠近任务**：取消当前 Nav2 目标并结束任务。

状态接口：`GET /status`，包含 `state`、`target_px`、`follow`、`follow_wait`、
`distance_m` 等字段。调试数据：`GET /debug`。

## 参数（`config/person_approach.json`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `standoff_m` | 0.3 | 车头到人的停车距离（米） |
| `robot_front_m` | 0.246 | 车头到 `base_footprint` 的距离 |
| `arrival_tolerance_m` | 0.02 | 到达判定容差 |
| `follow_replan_m` | 0.6 | 连续跟随：同一目标移动超过该距离才重新规划 |
| `follow_max_s` | 300 | 连续跟随总时长上限；0 表示不限制 |
| `multi_person_center_margin_px` | 32 | 多人时第一、第二目标的中心偏差小于该值则拒绝选择（按 640 宽换算） |
| `occlusion_grace_s` | 0.8 | 短暂漏检保留目标的时长 |
| `reacquire_timeout_s` | 8 | 超过该时长未找回同一目标则结束 |

## 状态

| state | 含义 |
|---|---|
| `preview_ready` / `ready` | 已生成候选路线 / 目标就绪 |
| `navigating` | 正在靠近 |
| `arrived` | 已到达（单次任务结束） |
| `arrived_waiting_for_target_move` | 连续跟随：到达并等待同一目标移动 |
| `ambiguous_person_choice` | 多人都在画面中心附近，拒绝静默选择 |
| `paused_waiting_for_target` | 导航中目标暂时丢失，已暂停并等待重捕获 |
| `target_reacquire_timeout` | 长时间未找回目标，任务结束 |
| `follow_timeout` | 连续跟随达到总时长上限 |

## 测试

```bash
cd tools
python3 -m unittest tests.test_person_approach tests.test_person_centering -v
micromamba run -n ros_jazzy python -m unittest tests.test_person_approach_ros -v
```

ROS 集成测试固定在 domain 79，使用假的 Nav2/雷达/TF，不会操作真实电机。
