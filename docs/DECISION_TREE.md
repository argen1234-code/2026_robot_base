# 分层决策树

## 1. 层级

```text
最上层决策层 task_coordinator
├── 导航栈决策层 nav_decision
├── 语音栈决策层 voice_decision
└── 视觉栈决策层 vision_decision（原 person approach 状态机 + 网页）
```

- 最上层负责调度、仲裁、任务生命周期和二次确认后的任务派发。
- 各栈决策层只执行本栈任务，并上报 `/decision/*_status`。
- 原始栈（导航、AI、视觉）可以独立启动；决策层为可选参数。
- 默认全栈启动时三类决策层全开；`--no-decision` 全关，也可逐栈关闭。

### 接口

| 方向 | 话题/服务 | 类型 |
|---|---|---|
| 语音→协调 | `/decision/set_task` | `robot_base_msgs/SetTask`（语音确认后调用） |
| 其他意图 | `/decision/intent` | `robot_base_msgs/DecisionIntent`（可选） |
| 协调→总状态 | `/decision/status` | `robot_base_msgs/TaskStatus` |
| 导航状态 | `/decision/nav_status` | `TaskStatus` |
| 语音状态 | `/decision/voice_status` | `TaskStatus` |
| 视觉状态 | `/decision/vision_status` | `TaskStatus` |
| 手动任务 | `/decision/set_task` | `robot_base_msgs/SetTask` |
| 导航任务 | `/decision/nav_task` | `SetTask` |
| 视觉任务 | `/decision/vision_task` | `SetTask` |
| 决策层说话 | `/decision/speak` | `std_msgs/String`（TRANSIENT_LOCAL，model_service TTS） |
| 目标净化 | `/goal_sanitizer/sanitize_goal` | `robot_base_msgs/SanitizeGoal` |
| 视觉网页 | `http://<pi>:8878` | `/status`、`/start`、`/follow`、`/stop` |

## 2. 优先级与抢占

集中在 `robot_base_decision/priority.py`：

```text
停止 stop                100  立即取消当前任务
手动目标 manual_goal      90  显式 Foxglove 目标，取消找人
已确认语音 confirmed_voice 70  二次确认后的语音任务
找人 follow               60  由语音/网页派生的视觉任务
空闲 idle                  0
```

规则：

- `/stop_request`、语音“停止”、网页“停止”最高优先级，立即取消。
- 手动 Foxglove 目标不需要语音确认，直接由 `nav_decision` 接管并取消找人。
- 手动目标到达后 2 s 内不再接受新的决策层导航任务，避免竞态。
- 新的语音任务在二次确认后可以抢占当前找人任务；未确认的提案不产生运动。
- 建图模式决策层关闭，只允许 Foxglove 手动设点。

## 3. 任务流程

### 3.1 “我要丢垃圾” / 来找我

```text
ASR 文本
  → voice_decision 识别 approach_user
  → 发布 /decision/voice_status = confirming
  → /decision/speak： “好的，那现在需要我去到你身边吗？”
  → 等待用户回答（正向：好的/是/可以/需要/确认；否定：不用/取消/停；超时 10 s）
      ├─ 否定/超时 → “好的，已取消”，结束
      └─ 确认 → voice_decision 调用 /decision/set_task（可靠服务）
            task=approach_user, execute/preview 由启动参数决定
            → task_coordinator
            → vision_decision 启动找人（vision_task=follow）
            → vision_decision 用雷达/相机锁定目标
            → 视觉决策层内部通过 Nav2 靠近（或由 nav_decision 配合）
            → 到达/失败 → /decision/vision_status 上报 → 协调层汇总
```

注意：

- 语音侧只提出任务，不直接发 `/goal_pose`。
- 决策模式启动时 AI 的 `SummonUserToolAdapter` / `MoveRelativeToolAdapter`
  不注册，避免绕过二次确认。
- 视觉任务需要 `--motor`、`calibration_confirmed=true`；否则只能预览。
- `approach_user` 在协调层映射为 `follow`（连续追踪）：视觉层启动后先
  缓慢旋转 360° 找人；找到人后用 bbox + 2D 雷达聚类锁定，导航中目标移动
  超过 `target_move_cancel_m` 会取消并重规划，到达停车距离后继续保持任务。
  测距只使用 `/scan_filtered`，深度相机不参与决策。

### 3.2 “向<方向>走<距离>米”

```text
ASR “向前走 2 米”
  → voice_decision 识别 move_relative {direction=forward, distance_m=2.0}
  → /decision/speak： “确认向前方走2米吗？”
  → 确认 → voice_decision 调用 /decision/set_task（可靠服务）
      task=move_relative → task_coordinator → nav_decision
      → nav_decision 用 TF 计算 map 目标
      → /goal_sanitizer/sanitize_goal（致命格/未知格检查）
      → NavigateToPose action
      → 结果上报 /decision/nav_status
```

否定或超时 → 取消，不发目标。

### 3.3 手动 Foxglove 目标

```text
Foxglove → /goal_pose → goal_sanitizer → /goal_pose_safe → bt_navigator
task_coordinator 监听 /goal_pose_safe
  → 取消 vision_decision 的找人任务
  → nav_decision 释放自己的 action
  → 手动目标独占导航 2 s
```

手动目标是显式操作，不需要二次语音确认。

### 3.4 停止

```text
/stop_request / 语音“停止” / 网页停止
  → task_coordinator 调 nav_task=stop + vision_task=stop
  → Nav2 action 取消，视觉任务取消
  → 底盘速度链归零；导航退出时 DISARM
```

### 3.5 建图模式

```text
--nav mapping
  → 决策层默认关闭
  → AI 可聊天，但 nav_goal_topic=/ai_nav_disabled
  → 只允许 Foxglove 手动设点建图
  → --decision / --task 与 mapping 同时出现直接报错
```

## 4. 失败分支

| 情况 | 行为 |
|---|---|
| 没有识别到目标人 | vision 状态 `no_person`，不运动 |
| 目标短暂丢失 | 保留目标位置，超时后结束 |
| 目标距离持续变远 | `moving_away_from_target`，立即取消 |
| 附近无合适停车点 | `no_free_standoff_goal`，不发送导航 |
| Nav2 目标被致命格拒绝 | `sanitize_goal` 失败，nav 状态 failed |
| 语音确认超时 | `timeout`，取消提案 |
| 手动目标到达 | 释放手动所有权，回到 idle |
| 决策层服务不可用 | fail closed，不运动 |

## 5. 默认启动与开关

| 场景 | 决策层 |
|---|---|
| 默认全栈（localize 最新地图） | 最上层 + 导航 + 语音 + 视觉全开 |
| `--nav localize --no-ai --vision none` | 导航决策层默认开；`--no-nav-decision` 关闭 |
| `--nav none --ai --vision none` | 语音决策层 |
| `--nav none --no-ai --vision astra` | 视觉决策层（需要导航才能执行找人，默认只预览） |
| `--nav mapping` | 全关，Foxglove 手动设点 |
| `--no-decision` | 全关，只跑原始栈 |
| `--task follow` | 启动并立即派发找人（需实车安全条件） |
| `--task preview` | 启动视觉预览，不运动 |
