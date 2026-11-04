# 语音召唤（垃圾桶找人）集成说明

本文档描述本次新增的“语音召唤 → 视觉定位 → Nav2 导航”任务层。

## 1. 新增/修改内容

| 文件 | 作用 |
|---|---|
| `largemodel/model_service.py` | 快速意图（停止/召唤）、视觉定位调度、TF/雷达缓存、导航目标发布 |
| `largemodel/nav_task.py` | 独立 Nav2 ActionClient 节点，异步执行 `NavigateToPose` |
| `largemodel/motion_gate.py` | Nav2 与底盘之间的速度门控，停止时持续输出零速度 |
| `utils/person_locator.py` | 人体框解析、方位角、地面投影、LiDAR 距离、停靠点几何 |
| `utils/tools_manager.py` | `locate_user()`：抓拍 + VLM + 测距 + map 目标 |
| `utils/tool_adapters.py` | `summon_user` 工具适配器（供 LLM/MCP 发现） |
| `config/yahboom.yaml` | 新增 `model_service`、`nav_task`、`motion_gate` 参数 |
| `launch/largemodel_control.launch.py` | 新增 `enable_nav` 参数和两个导航节点 |
| `launch/summon.launch.py` | `enable_nav:=true` 的便捷启动文件 |

## 2. 话题/动作契约

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/asr` | `std_msgs/String` | ASR → model_service | 语音识别文本 |
| `/text_response` | `std_msgs/String` | model_service → text_chat | 文字反馈 |
| `/summon_request` | `std_msgs/String` | model_service → 调试 | 召唤请求（信息性） |
| `/nav_goal` | `geometry_msgs/PoseStamped` | model_service → nav_task | `map` 坐标系下的停靠目标 |
| `/nav_enable` | `std_msgs/Bool` | nav_task → motion_gate | 校验通过的新任务才允许速度输出 |
| `/stop_request` | `std_msgs/Bool` | model_service → nav_task/motion_gate | 立即停止 |
| `/nav_status` | `std_msgs/String` | nav_task → model_service | 到达/失败等语音播报 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 → motion_gate | Nav2 速度输出 |
| `/cmd_vel` | `geometry_msgs/Twist` | motion_gate → 底盘 | 唯一速度出口 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | nav_task → Nav2 | 导航动作 |

## 3. 快速意图

`model_service` 在调用大模型之前先做确定性匹配：

- 停止：`停止`、`停下`、`停住`、`停一下`、`别动`、`不要动`、`停止导航`
- 召唤：`过来`、`来一下`、`来我这边`、`到我这里`、`垃圾桶过来` 等

匹配时先去掉唤醒词“你好小鑫”和标点，使用 `fullmatch`，避免把“我刚才过来的时候”误判为召唤。

停止路径会先发布 `/stop_request`，再播报“好的，已停止”，**不等待大模型**。

## 4. 视觉 → 目标

1. 用现有 USB 摄像头抓拍；
2. VLM 只返回一个 JSON：`{"boxes":[[x1,y1,x2,y2]]}`；
3. 恰好一个人、框合法才继续，否则拒绝移动；
4. 用框中心算方位角 `atan2(u-cx, fx)`；
5. 距离优先用 2D LiDAR 在方位角附近的点簇，失败则用地面投影（`z=0` 平面求交）；
6. 用**抓拍时刻**的 TF 转到 `map`；
7. 从人向机器人方向退 `standoff_distance_m` 得到停靠点，朝向人；
8. 通过 `/nav_goal` 发给 `nav_task`。

## 5. 安全行为

- 启动时 `motion_gate` 默认禁用；
- 只有 `nav_task` 校验目标后发布 `/nav_enable=true` 才允许速度通过；
- 停止时先禁用并发布零速度，再取消 Nav2 目标；
- 速度指令超过 `cmd_timeout_s` 未更新自动归零；
- 任务 ID 单调递增，迟到的旧 goal/result 不能重新使能；
- 可选 LiDAR 前向扇区急停（`lidar_safety`，默认关闭，确认雷达安装后再开）。

## 6. 启动

先启动导航项目（Cartographer/定位 + Nav2），再：

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom_ws/install/setup.bash

# 语音 + 召唤任务层
ros2 launch largemodel summon.launch.py

# 纯文字调试（不启动 ASR）
ros2 launch largemodel summon.launch.py text_chat_mode:=true
ros2 run text_chat text_chat
```

> 在 Docker 中运行时，路径是 `/root/yahboom_ws`，宿主机挂载目录是 `/home/pi/yahboom_ws`。

## 7. 联调命令

```bash
# 模拟语音“过来”
ros2 topic pub --once /asr std_msgs/msg/String "{data: '过来'}"

# 模拟语音“停下”
ros2 topic pub --once /asr std_msgs/msg/String "{data: '停下'}"

# 观察状态
ros2 topic echo /nav_status
ros2 topic echo /nav_goal

# 手动停止导航
ros2 topic pub --once /stop_request std_msgs/msg/Bool "{data: true}"

# 确认 Nav2 输出被重映射到 gate
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

## 8. 部署前必须完成的事项

1. **导航项目放进工作区**：`/home/pi/yahboom_ws/src/nav_xxx`（容器内 `/root/yahboom_ws/src/nav_xxx`）。
2. **安装 Nav2/Cartographer**：
   ```bash
   sudo apt install -y ros-humble-nav2-bringup ros-humble-cartographer-ros \
                       ros-humble-nav2-msgs ros-humble-nav2-simple-commander
   ```
3. **Nav2 速度重映射**：`controller_server` 和 `behavior_server` 的 `cmd_vel` 输出改到 `cmd_vel_nav`，最终由 `motion_gate` 发布 `/cmd_vel`。
4. **TF 链完整**：`map → odom → base_link → laser` 和 `map → odom → base_link → camera_optical_frame`。
5. **相机标定**：把 `camera_fx/fy/cx/cy` 换成实测值；`camera_frame` 必须是光学坐标系（x 右、y 下、z 前）。
6. **相机外参**：在 URDF/static_transform_publisher 中给出相机相对 `base_link` 的位置和俯仰角。
7. **串口命名**：麦克风和底盘不要都用 `/dev/ttyUSB0`，建议用 udev 固定为 `/dev/mic`、`/dev/base`。
8. **确认雷达**：`ros2 topic hz /scan`，确认 `laser` 坐标系名称与参数一致。
9. **资源**：Pi 5 8GB 上不要同时常驻 7B LLM + 3B VLM + Nav2 + Cartographer；建议意图模型 1.5B/3B，视觉模型按需加载，设置 `OLLAMA_MAX_LOADED_MODELS=1`。
10. **先验证固定点导航**，再接语音；不要在导航栈未稳定时整机联调。

## 9. 已知限制

- 只实现了“视觉定位 → map 目标”路径；**纯方位视觉伺服（bearing-only）尚未实现**。如果相机看不到人或没有可靠距离，会拒绝移动。
- 人不在相机视野内（另一个房间、背后）时无法主动寻找；需要后续增加旋转搜索/最后已知位置/声源定位。
- VLM 单帧推理较慢（文档数据约 8.5 s），当前实现只在召唤触发时调用一次，不做高频闭环。
- 地面投影假设地面平整、人脚可见；坐/躺场景建议依赖 LiDAR 或深度相机。
- `lidar_safety` 默认关闭，确认雷达前向无遮挡后再开启。

## 10. 本次实际完成与验证

已在容器 `ros_humble_ai_dev` 中完成：

- 安装 `ros-humble-nav2-msgs`、`ros-humble-nav2-bringup`、`ros-humble-cartographer-ros`、`ros-humble-nav2-simple-commander`、`ros-humble-robot-localization`；
- `colcon build --packages-select interfaces largemodel text_chat` 全部成功；
- `largemodel` 新增可执行文件 `nav_task`、`motion_gate`；
- 几何单测通过：人体框解析、方位角、四元数旋转、地面投影、LiDAR 方位测距、停靠点；
- 快速意图 11 组用例通过（“过来/停下”命中，普通句子不误触发）；
- `motion_gate` 功能测试通过：禁用时挡住速度、使能后转发、停止立即归零、停止后保持锁止、新任务可重新使能；
- `nav_task` 测试通过：无 TF 时拒绝、无 Nav2 时拒绝、停止不崩溃、错误路径不会误使能；
- `model_service` 启动测试通过：完成初始化并注册 7 个工具。

> 注意：安装依赖时容器曾因内存不足被 OOM 杀掉一次，现已重启容器并修复 dpkg 状态（`dpkg --audit` 为空）。原来正在运行的 `ros2 launch largemodel ...` 进程已随容器停止，需要重新启动。

## 11. 下一步需要你完成

1. 把导航项目放进 `/home/pi/yahboom_ws/src/`（容器内 `/root/yahboom_ws/src/`），`colcon build` 并 source。
2. 按导航项目的要求配置雷达、底盘串口、里程计/IMU 和 `map/odom/base_link/laser` 的 TF。
3. 标定 USB 相机内参，修改 `yahboom.yaml` 的 `camera_fx/fy/cx/cy`，并确认 `camera_frame` 是光学坐标系。
4. 在 Nav2 配置里把 `controller_server`、`behavior_server` 的 `cmd_vel` 输出重映射到 `cmd_vel_nav`。
5. 先单独验证：遥控底盘 → 建图 → 定位 → 固定点导航 → 停止。
6. 启动导航栈后，再运行 `ros2 launch largemodel summon.launch.py`，用 `/asr` 或 `text_chat` 触发“过来”。
7. 根据实测调整 `standoff_distance_m`、`max_user_distance_m`、`lidar_range_window_deg`；确认雷达安装后再打开 `lidar_safety`。
