# 运行手册

## 设备准备

```bash
# 雷达/STM32 的稳定设备名（按本机 USB 拓扑测量后修改 devpath）
sudo install -m 0644 \
  robot_ros2_ws/src/robot_base_bringup/udev/99-robot-base.rules \
  /etc/udev/rules.d/99-robot-base.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/ydlidar /dev/dap-vcp /dev/stm32
```

本机（树莓派，2026-11-05 实测）应看到：

| 软链接 | 来源 | 说明 |
|---|---|---|
| `/dev/ydlidar` | `ttyUSB0`（CP2102, 10c4:ea60） | YDLidar T-mini Plus，230400 |
| `/dev/stm32` | `ttyACM0`（0483:5740） | RobStride05 固件的原生 USB CDC，烧录本仓库固件后出现 |
| `/dev/dap-vcp` | `ttyACM1`（faed:4873） | Horco CMSIS-DAP v2 调试器的虚拟串口 |

> ⚠️ `ttyACM0` / `ttyACM1` 的编号由枚举顺序决定，会随插拔、重启变化；
> 不要写死编号。STM32 按 VID:PID `0483:5740` 匹配，调试器按物理口
> `KERNELS=="1-1.1"` 匹配，最终以 `/dev/stm32`、`/dev/dap-vcp` 为准。
> 换 USB 口或换机器时，用 `udevadm info -a -n /dev/ttyUSB0` /
> `-n /dev/ttyACM0` 重新取值并修改规则。

### 硬件识别自检

```bash
# 1) USB 设备是否存在（STM32 CDC / DAPLink / 雷达 CP2102）
lsusb | grep -E "0483:5740|faed:4873|10c4:ea60"

# 2) 稳定软链接是否生成
ls -l /dev/stm32 /dev/dap-vcp /dev/ydlidar

# 3) DAPLink v2 能否识别到 MCU（应看到 Cortex-M7 r1p1 / DPIDR 0x6ba02477）
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "init; targets; shutdown"

# 4) STM32 USB CDC 桥（不驱动电机）
python3 tools/usb_bridge_test.py

# 5) 雷达是否出数
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
ros2 topic hz /scan          # 另一个终端，应约 10 Hz
```

2026-11-05 实测结果：

| 检查项 | 结果 |
|---|---|
| STM32 原生 USB CDC | ✅ `0483:5740` → `/dev/stm32`（`ttyACM0`） |
| DAPLink v2 识别 MCU | ✅ CMSIS-DAPv2 `faed:4873`，DPIDR `0x6ba02477`，Cortex-M7 r1p1 |
| 激光雷达 | ✅ YDLidar Tmini Plus（Model Code 151），`/dev/ydlidar` |
| USB CDC 桥 | ✅ 遥测 ~50 Hz，IMU 在线，下行回显与 dry-run 轮速一致，ARM/DISARM 通过 |
| `/scan` | ✅ 10.035 Hz，430 点/圈（启动瞬间可能有少量强度位探测日志） |

> 用 OpenOCD/gdb 通过 SWD 读取变量时，MCU 会被 halt；读完后必须显式
> `resume`（或断开后重新上电），否则 USB 遥测会停、`usb_bridge_test.py`
> 会报 `no telemetry`。这不影响固件 Flash。

## Jazzy 导航环境（主机）

Debian 12 没有 ROS 2 官方二进制，本机使用 RoboStack conda 环境：

```bash
scripts/setup-jazzy-env.sh     # 首次创建 ros_jazzy 环境
scripts/build-nav.sh           # 编译 YDLidar SDK + robot_ros2_ws
```

进入环境并运行：

```bash
~/.local/bin/micromamba run -n ros_jazzy bash
source ~/2026_robot_base/robot_ros2_ws/install/setup.bash
ros2 launch robot_base_bringup robot_base.launch.py
```

> 交互 shell 也可以用 `micromamba activate ros_jazzy`。
> SDK 用系统 CMake 3.25 编译（conda 的 CMake 4.x 与老版 SDK 不兼容）。
>
> `~/.bashrc` 末尾已加入 `robot_base ROS 2 Jazzy auto-activate` 段：
> 新开的交互式终端会自动激活 `ros_jazzy`、source
> `robot_ros2_ws/install/setup.bash` 并设置 `ROS_DOMAIN_ID=1`。
> 不需要自动加载时注释掉该段即可（原始备份在 `~/.bashrc.bak.*`）。

## USB CDC 桥（底盘 ⇄ 树莓派）

固件在 `stm32-firmware/RobStride05_H743`：

- `App/app_usb.c`：下行 `AA 55` 速度帧 / `BB 55` ARM 服务帧，上行 `CC 55`
  104 字节遥测（轮速、目标、电流、JY901S 的 roll/pitch/yaw、陀螺、加速度）。
- `USB_DEVICE/` + `Middlewares/ST/STM32_USB_Device_Library/`：STM32 USB Device
  栈（PID 0483:5740，枚举为 `/dev/stm32`）。

构建与烧录（DAPLink / CMSIS-DAP v2）：

```bash
stm32-firmware/RobStride05_H743/gcc/build.sh
stm32-firmware/RobStride05_H743/gcc/flash.sh
lsusb | grep 0483:5740 && ls -l /dev/stm32
```

## 统一启动（推荐）

整个工程只需要一个入口，用 `slam` 切换：

```bash
# 建图模式：默认用 ~/maps 最新 posegraph 续建，没有则从零建图
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=true motor_enable:=true visualization:=foxglove

# 先验地图模式：slam_toolbox localization + Nav2
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:=/home/pi/maps/robot_map \
  motor_enable:=true visualization:=foxglove
```

- `map` 是 **posegraph 前缀**，不是 `.yaml/.pgm`；
- `slam:=true`：默认加载 `~/maps` 最新 `*.posegraph` 续建；
  没有 posegraph 则从零建图；`map:=<prefix>` 指定地图，`map:=none` 强制新图；
- `slam:=false`：默认用最新 posegraph；没有则在启动节点前报错；
- **先验地图重定位使用 slam_toolbox localization**，不再使用 AMCL/map_server；
  初始位姿由 `map_start_pose`（默认 `[0,0,0]`）或 `map_start_at_dock` 决定；
- 保存 posegraph（建图栈运行中）：
  ```bash
  bash tools/save_slam_map.sh /home/pi/maps/robot_map
  ```
  `map_saver` 的 YAML/PGM 不能用于 slam_toolbox localization。

## 主机一键启动与手部检测（2026-11-05）

主机原生栈（不经过 Docker/MQTT），导航与语音同 `ROS_DOMAIN_ID=1`：

```bash
cd ~/2026_robot_base
bash scripts/start_host_stack.sh                # 干跑（不 ARM 电机）
bash scripts/start_host_stack.sh --motor        # 允许电机
bash scripts/start_host_stack.sh --no-hand      # 关闭手部检测
bash scripts/start_host_stack.sh --viz foxglove # 开 Foxglove（8765）与压缩图
```

- 日志：`~/rosbags/host_stack_logs/<时间戳>/`（`nav.log` / `ai.log` /
  `hand_det.log` / `hand_bridge.log`）
- Ctrl-C 统一停止导航、AI、手部检测；导航退出时自动 DISARM
- 手部检测在 `/dev/hand-cam` 与 `/home/pi/mediapipe_plain` 都存在时默认启用
- 相机稳定别名（USB 枚举顺序会变，不要用 `/dev/videoN`）：
  `/dev/hand-cam`（icSpring）、`/dev/astra-color`（Astra 彩色）。
  安装规则：
  ```bash
  sudo install -m 0644 deploy/udev/99-cameras.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=video4linux
  ls -l /dev/hand-cam /dev/astra-color
  ```

手部链路（检测器不导入 ROS，走本机 UDP）：

```text
/dev/hand-cam → MediaPipe 独立 venv (/home/pi/mediapipe_plain)
            → UDP 127.0.0.1:5005
            → tools/hand_pose_bridge.py (ai_host, domain 1)
            → /perception/hand_pose   geometry_msgs/PoseStamped
            → /perception/hand_debug  std_msgs/String (JSON)
```

验证：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /perception/hand_debug
ROS_DOMAIN_ID=1 ros2 topic echo /perception/hand_pose
```

`hand_debug` 字段：`valid`、`message`、`bbox_px`、`center_px`、`cartesian_m`、
`bearing_rad`、`distance_m`、`fps`。检测器日志的 `hand detected/lost`
可用来确认相机和模型是否工作。

### Foxglove 查看两路相机（压缩图，10 Hz）

```bash
bash scripts/start_host_stack.sh --viz foxglove
# Foxglove Studio 连接 ws://<树莓派IP>:8765
```

Image 面板订阅（均为 `sensor_msgs/CompressedImage`，jpeg）：

| 话题 | 来源 | 实测 |
|---|---|---|
| `/camera/color/image_raw/compressed` | model_service（Astra 彩色 `/dev/astra-color`） | 640×480，约 18 KB/帧 |
| `/perception/hand_image/compressed` | 手部桥接（icSpring `/dev/hand-cam`） | 640×480，约 45 KB/帧 |

- 两路合计约 5 Mbps，WiFi 可承载；**不要订阅 raw `Image` 或
  `PointCloud2`**，10 Hz 下会打满链路
- 彩色图由 `publish_compressed_image`（`yahboom.yaml`）控制，且只在有订阅者
  时抓帧编码；无人观看时零开销
- `--no-cam-images` 关闭手部图转发；`--cam-images` 在 `--viz none` 时也强制开启
- 手部 JPEG 走本机 UDP 5006，检测器侧可用 `--image-fps` / `--jpeg-quality` 调节

重建隔离 venv（主机 Jazzy 的 numpy 2.x / OpenCV 5 与 MediaPipe 不兼容）：

```bash
python3 -m venv /home/pi/mediapipe_plain
/home/pi/mediapipe_plain/bin/pip install -U pip
/home/pi/mediapipe_plain/bin/pip install mediapipe opencv-python "numpy<2" \
  packaging setuptools "protobuf<5" pyparsing python-dateutil
```

Orbbec Astra Pro 深度（主机 Jazzy 驱动，color 走 `/dev/astra-color` 原始 UVC）：

```bash
ros2 launch orbbec_camera astra.launch.py enable_color:=false \
  enable_depth:=true enable_point_cloud:=false depth_format:=Y11 depth_fps:=30 \
  depth_width:=320 depth_height:=240
ROS_DOMAIN_ID=1 ros2 topic echo /camera/depth/camera_info
```

> 人像检测只需要深度图，默认关点云。**Astra Pro 在 USB2.0 上 640×480 Y11@30
> 实测只有 18–24 fps、单帧间隔可达 180 ms（画面明显卡顿）；320×240@30 实测
> 稳定 29.7 fps，驱动 CPU 约减半。** `start_host_stack.sh` 已默认 320×240，
> 需要高分辨率时用 `DEPTH_WIDTH=640 DEPTH_HEIGHT=480` 启动，但要接受掉帧。
> 需要 3D 点云时再 `enable_point_cloud:=true depth_fps:=30`。
> 注意：彩色、深度、手部相机都在 Bus 01（480 Mbps）上，与 color UVC 同时开时会
> 进一步挤占带宽。

## YOLO 人像检测（深度相机，2026-11-05）

模型（一次性，约 13 MB，仓库不提交）：

```bash
bash scripts/fetch-yolo.sh        # -> ~/models/yolov8n.onnx（SHA1 校验）
```

启动（`--person` 会自动拉起 Orbbec 深度）：

```bash
bash scripts/start_host_stack.sh --viz foxglove --person
# 只起深度：--depth
# 人像仅 2D：--person --no-depth
```

话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/person_detections` | `String` JSON | bbox、conf、`depth_m`、`bearing_deg`、`xyz`、`infer_ms` |
| `/perception/person_pose` | `PoseStamped` | 最近的人，`camera_depth_optical_frame` |
| `/perception/person_image/compressed` | `CompressedImage` | 标注图（Foxglove Image 面板） |
| `/camera/color/image_raw/compressed` | `CompressedImage` | 彩色源（model_service） |
| `/camera/depth/image_raw` | `Image` 16UC1 | 深度源（Orbbec） |

说明：

- YOLOv8n ONNX 640×640，OpenCV DNN；只取 person 类，`conf>0.35`、NMS 0.45。
- Pi 5 实测：空闲约 0.28 s/帧；整栈满载约 0.7–1.0 s/帧（节点上限 3 FPS）。
- 彩色/深度未严格标定：在 bbox 内缩 20% 取有效深度（>300 mm）的 30% 分位数；
  偏差时用 `--depth-offset-x/y` 微调。
- Orbbec 的 `camera_info` 在本机 K 为 NaN，节点默认 `fx=fy=570`、
  `cx=319.5`、`cy=239.5`。
- 深度镜头被挡/贴得太近时会限频告警
  `person bbox has no valid depth`；把相机移到无遮挡、0.5–3 m 视野即可。
- 模型下载失败时检查网络；脚本会依次尝试 ghfast 代理和 GitHub 直连。

## 语音唤醒与识别验证（2026-11-05）

唤醒词由 KWS 串口模块识别（`/dev/mic-kws`，CH340，115200）。流程：

```text
说“你好小亚”
→ KWS 串口帧 → /wakeup (Bool) + 中文提示音（longwan-women-1.mp3）
→ 蜂鸣提示后开始录音（arecord plughw:2,0 → 16 kHz，VAD 断句）
→ 云端 Paraformer → /asr (String) + 日志打印识别文本
→ model_service：DeepSeek/工具 → /text_response + 云端 TTS 播报
```

验证命令（domain 1）：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /wakeup
ROS_DOMAIN_ID=1 ros2 topic echo /asr            # 最终识别文本
ROS_DOMAIN_ID=1 ros2 topic echo /text_response
tail -f ~/rosbags/host_stack_logs/<时间戳>/ai.log
```

日志关键字：`kws detected` → `I'm here` → `okay😀` → 识别文本。

KWS 不触发时，绕过唤醒词直接验证录音/识别：

```bash
ROS_DOMAIN_ID=1 ros2 topic pub --once /wakeup_trigger \
  std_msgs/msg/Bool "{data: true}"
```

然后说一句话，观察 `/asr` 与日志中的 VAD 帧（`1`/`-`）。

设备与实现说明：

- 麦克风是 USB Audio（card 2），只支持 44.1 kHz；代码用
  `arecord -D plughw:2,0 -r 16000` 采集（plug 插件重采样），
  参数为 `mic_alsa_device`。不要改回 PyAudio：ai_host 的
  PortAudio/ALSA 插件不全会失败或段错误。
- 提示音用 pygame 播放；`playsound` 在主机上会调用系统 Python 并缺
  GStreamer 依赖。
- 在线 TTS 输出写到 `/tmp/robot_tts_output.mp3`，不再写仓库目录
  （避免容器以 root 创建文件后主机 pi 无法覆盖）。
- “我还不理解你的意思”是 **ASR 空结果/过短** 的预录提示音，不是大模型
  拒答。先用 `ros2 topic echo /asr` 看实际识别文本；说完整短句
  （如“后退一米”）比长句更稳。
- 本机 KWS 模块的唤醒词是**“你好小亚”**（固件决定）。若说唤醒词
  日志没有 `kws detected`，需要重烧模块；可先用 `/wakeup_trigger`
  验证后半段链路。

业务空间专属域名（新版 `sk-ws-` Key 必须使用，`.env` 已配置）：

```bash
DASHSCOPE_BASE_URL=https://ws-xbgkdi9pjur8a9b6.cn-beijing.maas.aliyuncs.com/api/v1
DASHSCOPE_WS_URL=wss://ws-xbgkdi9pjur8a9b6.cn-beijing.maas.aliyuncs.com/api-ws/v1/inference
DASHSCOPE_OPENAI_BASE_URL=https://ws-xbgkdi9pjur8a9b6.cn-beijing.maas.aliyuncs.com/compatible-mode/v1
```

- `sk-ws-` 是**业务空间 Key**，配旧共享域名 `dashscope.aliyuncs.com`
  会返回 `AccessDenied.Unpurchased`；必须配上面的专属域名。
- 旧 `sk-` Key + 旧共享域名仍可用，两套 Key/域名**不能混用**。
- 业务空间 ID：`ws-xbgkdi9pjur8a9b6`（默认业务空间，华北2·北京）。

常见错误：

| 现象 | 原因 |
|---|---|
| `/wakeup` 有、但 ASR 报 `Access to model denied` / TTS 报 `Arrearage` | DashScope 账户欠费或未开通 paraformer / cosyvoice，检查阿里云费用中心与模型服务 |
| `AccessDenied.Unpurchased` | Key 与 Base URL 的业务空间/套餐不匹配：`sk-ws-` Key 必须配 `DASHSCOPE_*` 专属域名，不能用旧共享域名 |
| `Failed to open KWS serial port` | `/dev/mic-kws` 不存在；安装 `deploy/udev/99-mic-kws.rules` |
| 有 `I'm here` 但 `/asr` 无输出 | 看 `ai.log` 的 `Microphone capture` / `ASR Error`；确认 `plughw:2,0` 可被 arecord 打开 |
| 说了唤醒词毫无反应 | 模块唤醒词不匹配，或 `/dev/mic-kws` 未接；用 `/wakeup_trigger` 区分 |

## 底层入口（调试用）

`navigation.launch.py` 现在也是 slam_toolbox localization + Nav2；
直接运行需要自己提供 `slam_params_file`，一般用上面的统一入口。

- `chassis_bridge` **始终启动**，负责遥测；`motor_enable:=false`（默认）时
  仍发零速帧、上传播遥测，但不会 ARM/使能电机。
- `motor_enable:=true` 时节点每秒请求 ARM；固件完成双电机 PREPARE+ENABLE
  后才接受速度帧。节点退出（Ctrl-C）会发送 DISARM。
- 固件看门狗：ARM 后 500 ms 没有合法速度帧就把目标清零（保持 ARM）。

话题：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/chassis/imu` | `sensor_msgs/Imu` | JY901S，frame_id=`base_link`；已减静止零偏，绝对 yaw 会漂，勿作航向 |
| `/chassis/wheel_speed` | `std_msgs/Float32MultiArray` | `[left, right]` rad/s，前进为正 |
| `/chassis/wheel_target` | `std_msgs/Float32MultiArray` | `[left, right]` 目标轮速，rad/s |
| `/chassis/wheel_current` | `std_msgs/Float32MultiArray` | `[left, right]` 相电流，A；用于诊断打滑/扭矩 |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | Nav2 速度链末端（下行） |
| `/remote_cmd_vel` | `geometry_msgs/Twist` | 遥控输入，mode=2（下行） |

台架测试（默认不驱动电机，只验证链路/遥测/ARM）：

```bash
python3 tools/usb_bridge_test.py
```

> 两轮 CAD 模型（2026-11-04）：几何轮半径 **0.041 m**、有效轮距 **0.148 m**；
> 2026-11-05 卷尺/编码器/雷达标定有效滚动半径 **0.041713 m**。
> 固件 `APP_WHEEL_RADIUS_M`、`controllers.yaml` 的 `wheel_radius` 用 0.041713；
> URDF 视觉网格保持几何 0.041 m。`APP_TRACK_WIDTH_M` / `wheel_separation`
> 仍为 0.148。协议字段以 `chassis_protocol.py` 顶部注释和 `app_usb.h` 为准。

## 2026-11-05 标定记录

- **模型/TF**：车体保持 CAD 朝向，
  `base_footprint → base_link = (0.075, 0, 0.103), yaw 0`；
  `laser_frame` 的 yaw 从 `-90°` 修正为 **`+90°`**：
  raw `laser_frame -90°` 对应 `base_link` 车头 `+X`，raw `+90°` 对应车尾
  （自回波方向）。雷达在 `base_footprint +X` 端，约 +0.2225 m。
  `navigation_footprint` 圆心保持 `+0.1005`，Nav2 footprint 多边形为
  `[[-0.045,-0.118],[-0.045,0.118],[0.246,0.118],[0.246,-0.118]]`。
- **键盘测试**：W/S/A/D 与实车完全一致；A 左转 `Δyaw +17.3°`、
  `gyro_z +9.84°/s`，D 右转 `Δyaw -19.2°`、`gyro_z -10.43°/s`
  → IMU 轴符号与 ROS REP-103 一致，左右轮映射无需交换。
- **IMU 零偏**：静止 baseline `roll +2.005°`、`pitch -0.033°`、
  `yaw +218.260°`、陀螺零偏 0；已写入
  `config/imu_calibration.yaml` 并由 `chassis_bridge.py` 减去。
  ⚠️ 连续行驶 13 s 时 JY901S 融合 yaw 漂了约 **97°**，但 `gyro_z`
  均值仅 `-0.104°/s`、雷达也显示车体直行 → **绝对 yaw 不可作为航向**，
  短时航向请用 `gyro_z` 积分或激光里程计。
- **速度标定**：0.10 m/s × 12.88 s，目标左右轮各 5 圈；
  编码器均值 **4.998 圈 / 1.2876 m**，卷尺 **1.31 m**；
  雷达正前方箱体（raw -105°）`2.758 → 1.439 m`（Δ **1.319 m**）。
  有效滚动半径 **0.041713 m**（`1.31 / (2π × 4.998)`），相对编码器比例
  `1.0174`。已改固件 `APP_WHEEL_RADIUS_M`、`controllers.yaml` 并烧录，
  结果保存在 `config/speed_calibration.yaml`。
- **工具**：
  - `python3 tools/keyboard_cmd.py`：W/S/A/D 直连 + IMU/轮速逐帧记录
  - `python3 tools/imu_calibrate.py --guided`：静止/姿态引导 IMU 轴测试
  - `bash tools/run_speed_calibration.sh --revs 5 --speed 0.10`：
    雷达 + 固定圈数速度标定

## 建图导航 1 m 目标链路自检（2026-11-05）

修正 `laser_frame` yaw=+90° 后，用“雷达正前方 1 m”目标验证整条链路：

```bash
# 终端 A：建图 + Nav2 + Foxglove
ros2 launch robot_base_bringup mapping_navigation.launch.py \
  motor_enable:=true visualization:=foxglove

# 终端 B：录包（先开，再发目标）
ros2 bag record -o /home/pi/rosbags/mapping_nav_$(date +%Y%m%d_%H%M%S) \
  /scan /scan_filtered /map /tf /tf_static \
  /odom /odom_rf2o /odom_stabilized /wheel/odom \
  /cmd_vel /cmd_vel_smoothed /plan /local_plan /behavior_tree_log \
  /chassis/imu /chassis/wheel_speed /chassis/wheel_target /goal_pose

# 终端 C：发目标（等 map TF 出现后，只发一次）
python3 tools/send_goal_ahead.py --distance 1.0
```

`send_goal_ahead.py` 从 `map→base_footprint` 读当前位姿，按车头方向
把目标放在“雷达前方 1.0 m”= `base_footprint` 前方约 1.2225 m，
发布到 `/goal_pose`（Nav2 `bt_navigator` 直接订阅）。
**只发一次**：重复发布会让每个新目标 preempt 上一个，控制器只跑一小段
（2026-11-05 录包实测：5 次目标 → 0.8 s 指令、车只走 4 cm）。

预期：Foxglove 中 `/plan` 从机器人指向车头方向；车应向前走约 1.2 m。
若路径指向车尾或车反向走，立即 Ctrl-C，检查 `laser_frame`/底盘符号。

2026-11-05 实测（`/home/pi/rosbags/mapping_nav_20261105_024309`）：
一次目标、`/cmd_vel_smoothed` 6.65 s、最大 0.20 m/s；`/odom` 位移
**1.126 m**（目标 1.222 m），终点距目标 0.101 m < `xy_goal_tolerance
0.18`，yaw -8.9° < 0.25 rad；BT 重规划全部成功、无 abort。
重复发 `/goal_pose` 会 preempt 上一个目标，表现为“车只动一小段”，
`send_goal_ahead.py` 已改为只发一次。

干跑（`motor_enable:=false`）已验证：TF `map→base_footprint` 可查、
`/goal_pose` 被接收、`bt_navigator`/`controller_server` 进入导航；
没有 `chassis_bridge` 时控制器会因无进度 abort，这是预期。

## EKF 松耦合里程计（2026-11-05）

`/odom` 现在由 `robot_localization` 的 `ekf_filter_node` 发布，融合三路：

| 输入 | 话题 | 融合字段 |
|---|---|---|
| 激光 + ZUPT | `/odom_stabilized` | `x, y, yaw` |
| 轮式 | `/wheel/odom` | `vx, vyaw` |
| IMU | `/chassis/imu` | 仅 `gyro z`（vyaw） |

TF 归属：rf2o 和 `odom_stabilizer` 都 `publish_tf=false`；
`wheel_odom` 不发 TF；只有 EKF 发 `odom → base_footprint`，
`slam_toolbox` 继续发 `map → odom`。

- 配置：`config/ekf.yaml`，`frequency: 30`，`two_d_mode: true`；
- 轮式：有效轮径 **0.041713 m**、轮距 **0.148 m**；
- IMU 绝对 yaw 在电机运行时漂移，**不融合**，只融合陀螺 z；
- 静止 8 s 实测 EKF `/odom` 漂移约 **0.0000 m / 0.0000°**；
- 1 m 目标测试 `/odom` 到 **x≈1.04 m**；输出约 **30 Hz**；
- Pi 满载时偶发 `Failed to meet update rate`，可把频率降到 20 Hz。

## 静态障碍调参（rosbag）

当前静态 bag：`/home/pi/rosbags/static_opt_segment_20261105_040149`
（从最新 mapping_opt bag 前 18 s 提取，`/scan` 155 帧、15.3 s）。

> 旧的脉轮模型 `static_tune_20261104_214418` 和旧自回波
> `self_echo_new_20261104_233247` 已清理。
>
> 自回波结论保留：全部 <0.10 m，持续波束 raw `+86°..+104.5°`
> （经 TF 对应 `base_link` 车尾）；`laser_filters.yaml` 现用
> 0.20 m 距离截断，并把 raw `+80°..+110°` 置 NaN。
> 需要复测自回波时重新录一段静止 `/scan`，再运行
> `python3 tools/self_echo_measure.py --bag <bag>`。

```bash
bash tools/run_costmap_tuner.sh --no-viewer       # 默认，无 GUI
bash tools/run_costmap_tuner.sh --rviz            # 尝试启动 RViz
bash tools/run_costmap_tuner.sh --legacy-tf       # 回放旧脉轮 bag
bash tools/run_costmap_tuner.sh --no-viewer --duration 20
```

调参栈：回放 `/scan` → `laser_filters` → `/scan_filtered_tuned` →
独立 `nav2_costmap_2d`（节点名 `costmap_tuner`）→ `/costmap`。
参数在 `robot_ros2_ws/src/robot_base_bringup/config/costmap_tune.yaml`。

可视化说明：

> ⚠️ 本机 RoboStack aarch64 的 `rviz2`（14.1.24）在
> `Ogre::GLRenderSystemCommon::initConfigOptions()` 必段错误；系统 GLX（V3D，
> OpenGL 3.1）正常，换软件 GL / profile 覆盖都无效。因此默认用
> **Foxglove**（WebSocket，不依赖本机 OpenGL）：
>
> ```bash
> ros2 launch robot_base_bringup mapping_navigation.launch.py \
>   motor_enable:=true visualization:=foxglove
> # 浏览器打开 https://studio.foxglove.dev
> # 连接 ws://<树莓派IP>:8765
> ```
>
> 3D 面板设置 → Publish → Type `Pose`、Topic `/goal_pose`（Fixed frame `map`），
> 再用工具栏的 **Publish pose** 在地图上点目标（等同 RViz 的 2D Goal Pose）。
> 机器人模型要单独加「自定义图层 → 添加 URDF」：Source 选 Topic
> `/robot_description`（或 Parameter `/robot_state_publisher.robot_description`），
> Control mode `Transforms`、Display mode `Visual`。蓝色圆是
> `/local_costmap/published_footprint` 多边形，不是模型。
> STL 默认按 Y 轴向上加载；模型“躺着”时在 3D 面板 → 场景 →
> **网格上轴（Mesh up axis）** 改成 **Z-up**。
> 也可单独起桥：`ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765`。
> 调参脚本默认不启动 GUI，可用 `--rviz` 打开 `config/costmap_tune.rviz`；
> 否则用 `ros2 topic hz /costmap` 和 `tools/costmap_stability.py` 观察/量化。

实测结论：

- 机器人确实静止（原始扫描首末帧平均差 4.7 mm），但 rf2o 在 34 s 内漂了
  12.3° yaw。`odom_stabilizer.py` 用轮速做零速保持后，静止漂移显著降低；
  2026-11-05 起 `/odom` 再由 EKF 融合轮速/IMU，静止 8 s 漂移
  **0.0000 m / 0.0000°**。
- 激光滤波：0.20 m 距离截断去掉 <0.10 m 自回波；`laser_frame` yaw
  +90° 修正后 raw `+86°..+104.5°` 对应 `base_link` 车尾，另加
  raw `+80°..+110°` 角度掩码（置 NaN），车头 raw ≈ -90° 保持有效。
  该掩码是 2026-11-05 新增，替代此前“后向不需要遮罩”的结论。
- 局部代价地图：3×3 m、0.03 m、10 Hz、`obstacle_min_range=0.20`、
  `inf_is_valid=false`。播包期间致命格连续帧 Jaccard ≈ 0.86，质心标准差
  ≈ 1.8 cm；bag 结束后 168 个致命格完全不变（Jaccard = 1.000）。
- 正式参数已同步到 `config/nav2_params.yaml` 的 `local_costmap` 段和
  `config/laser_filters.yaml`。

## 建图 bag 回放调参记录（2026-11-05）

建图 bag：`/home/pi/rosbags/mapping_opt_20261105_035551`（254 s、84 帧 `/map`）：

- `map_stability.py`：Jaccard mean **0.9691**、min **0.5507**、retention **0.8337**；
- 低 Jaccard 帧（14/18/34/41/47/58/73）对应地图扩边/回环修正，
  `lost_occ ≈ gained_occ`，优先判断为回环重优化，不一定是漂移；
- 最终地图 occupied 2164 格。

从该 bag 前 18 s 提取静止段：`static_opt_segment_20261105_040149`
（`/scan` 155 帧、15.3 s），用于 costmap 回放。

干净单实例基线（`+66..+114°` 掩码、`min_range=0.20`）：

- lethal count 85–95，mean 89.7；
- 连续帧 Jaccard mean **0.873**、min **0.758**、max 1.000；
- 质心标准差 x **11.9 mm**、y **23.1 mm**。

四轮纯离线 SLAM 回放（同一 bag、同一 `/odom`、rate 2）：

| 轮次 | 变化 | occupied | 连通域 | 孤立域 | precision | recall | unique |
|---|---|---:|---:|---:|---:|---:|---:|
| R1 | 基线 `+66..+114°` | 2225 | 194 | 76 | 0.9155 | 0.9497 | 188 |
| R2 | 节点 0.3 m/0.3 rad | 2003 | 210 | 87 | 0.7124 | 0.6653 | 576 |
| R3 | resolution 0.03 | 3772 | 375 | 206 | 0.7746 | 0.7174 | 506 |
| R4 | 掩码 `+80..+110°` | 2199 | 194 | 75 | 0.9177 | 0.9408 | 181 |

结论：R2/R3 明显增加孤立域和 unique 噪声，不采用；R1/R4 很接近，
R4 的孤立域/unique 略少、precision 略高，最终采用 **`+80..+110°`**
（R4），但 R1 的 recall 略高，实车复跑时保留 R1 作为对照。

试过但未采用：

- 3 点空间中值滤波：质心抖动和 Jaccard 变差；
- `min_range=0.25`：Jaccard/质心变差；
- 多次回放时残留多个 `costmap_tuner` 会污染 `/costmap` 指标，后续必须
  单实例运行；`run_costmap_tuner.sh` 的静态 TF 已修正为
  `laser_frame yaw +90°`；
- R2 的节点稀疏化和 R3 的 0.03 m 分辨率地图噪声更大，不采用。

工具：`tools/replay_mapping.sh` 纯离线 SLAM 回放、
`tools/compare_maps.py` 地图指标对比。

下一步：用 `+80..+110°` 掩码实车复跑同一路线，与 R1/R4 离线地图和
`map_stability` 对照；costmap 仍用单实例回放调参。

## Nav2 鲁棒性策略（2026-11-05）

不是只调 Nav2 参数，增加了目标点安全节点和恢复策略：

1. **`goal_sanitizer.py`**
   - 用户/Foxglove 仍只发 `/goal_pose`；
   - sanitizer 检查后发布 `/goal_pose_safe`，`bt_navigator` 订阅后者；
   - 终点 cost ≥90 或未知：在 `max_goal_adjust_m=1.0` 内搜索最近的自由格
     （cost ≤20），保持朝向，替换目标；1 m 内没有自由格则拒绝并在
     `/goal_sanitizer_status` 输出原因；
   - 起点 cost ≥90：调用 local/global costmap 的 `clear_entirely` 服务，
     然后交给 BT 的恢复链。
   - 建议 Foxglove Fixed frame 用 `map`；若用 `base_link`，目标 stamp 查 TF
     失败时会回退到最新 TF（2026-11-05 修复），不会再直接丢弃。
2. **BT 恢复链**
   - 当前使用 Nav2 默认 BT，已实测 `bt_navigator` active；
   - 默认 BT 自带清图、`Spin`、`Wait`、`BackUp` 恢复；起点真障碍时
     清图失败后由 `BackUp` 退出再重规划；
   - 参考仓库 `2026_new_sentry` 的自定义 BT 在当前 Jazzy 的
     `bt_navigator` 上会报 `ID [ComputePathToPose] already registered`，
     不能直接加载；`behavior_trees/` 里保留了两份 XML 作为参考，
     运行时不要覆盖 `default_nav_to_pose_bt_xml`。
3. **验证**
   - `bt_navigator` active，`/goal_pose_safe` 的订阅者是 `/bt_navigator`；
   - 用地图边缘墙内点测试：`goal_sanitizer` 正确拒绝并输出
     `终点 cost=100...1.0 m 内无自由格，拒绝目标`；
   - 目标在墙内但 1 m 内有自由格时会自动修正到最近自由格。

说明：`max_goal_adjust_m`、`free_threshold`、`lethal_threshold` 是
sanitizer 节点的业务参数，不属于 Nav2 调参；参考仓库的
`back_up_free_space` 插件未移植，当前使用标准 `BackUp`。

## 离线 Nav2 回放调参（2026-11-05）

工具链（都在 `tools/`）：

- `bag_replay_relay.py`：把 bag 的 `/scan_filtered`、`/odom`、`/tf`、`/goal_pose`
  重打当前时间戳后转发，避免旧 TF 时间戳让 Nav2 认为数据过期；
- `nav2_replay_monitor.py`：采集实际频率/最大间隔、速度、BT 失败与成功、
  每目标耗时、CPU/内存/温度；
- `run_nav2_replay.sh`：只启动 Nav2（不含 SLAM/EKF/底盘），回放固定 bag；
- `nav2_replay_rounds.py` / `nav2_replay_report.py`：8 轮参数矩阵与汇总。

```bash
bash tools/run_nav2_replay.sh \
  --params <yaml> --out /home/pi/rosbags/nav2_replay/Rx
python3 tools/nav2_replay_rounds.py --from R1 --to R8
python3 tools/nav2_replay_report.py <各轮 metrics 目录...>
```

输入 bag：`/home/pi/rosbags/localization_20261105_055426`（211 s、6 个目标）。
回放栈只跑 Nav2，`/odom`、`/tf`、`/scan_filtered` 来自 bag，因此**机器人轨迹是
录制的，不会随新指令变化**：可用于验证实际频率、CPU、指令形状，
**不能**验证“更快到达或更少失败”，后者必须实车。

### 关键修正：局部代价地图“6 Hz”是发布，不是更新

- `/local_costmap/published_footprint` = **10 Hz**（跟随 `update_frequency`）；
- `/local_costmap/costmap` 只有约 **6.3 Hz**，这是全量 OccupancyGrid 的发布限速
  （主要供可视化）；控制器进程内用的是 10 Hz 更新后的代价地图。

把局部 `update_frequency` 提到 12 Hz 后：更新 **12 Hz**、发布约 **7.5 Hz**，
控制器实际频率和目标耗时均无变化。

### 8 轮结果（同 bag、同目标）

| 轮次 | 改动 | 控制实际 | 地图更新/发布 | 目标耗时（s） | FollowPath 失败 |
|---|---|---:|---|---|---:|
| R1 | 基线 | 13.64 Hz | 10 / 6.3 | 39.0/16.7/23.2/25.6 | 4 |
| R2 | 鲁棒参数 + 局部射线 3 m | 13.65 Hz | 10 / 6.28 | 40.0/15.7/23.4/25.6 | 4 |
| R3 | 局部 update 12 Hz | 13.64 Hz | 12 / 7.28 | 39.7/15.9/23.4/25.7 | 4 |
| R4 | 控制 18 Hz + 平滑 25 Hz | 16.36 Hz | 12 / 7.51 | 39.7/15.9/23.4/25.7 | 4 |
| R5 | 控制 20 Hz | 18.16 Hz | 12 / 7.51 | 39.2/16.5/23.3/25.6 | 4 |
| R6 | 转弯参数（角加速度 0.9 等） | 18.16 Hz | 12 / 7.74 | 39.7/15.9/23.3/25.6 | 4 |
| R7 | 局部膨胀 0.40 / 6.0 | 18.17 Hz | 12 / 7.55 | 39.1/16.6/23.3/25.6 | 4 |
| R8 | DWB 加减速 0.35 / -0.5 | 18.18 Hz | 12 / 7.47 | 39.1/16.6/23.3/25.6 | 4 |

CPU 均值全程约 59–63%，内存峰值约 480–550 MB（R2 含重复
`goal_sanitizer`，数据偏高）。

### 已落地（低风险、回放验证过）

- `controller_frequency: 15 → 18`；`smoothing_frequency: 20 → 25`：
  实际输出 13.6 → 16.4 Hz、18.3 → 22.8 Hz，CPU 基本不变；
- 局部代价地图 `update_frequency: 10 → 12`（`publish_frequency` 保持 10）；
- 局部障碍层 `raytrace_max_range` / `obstacle_max_range: 8/6 → 3/3 m`
  （3×3 m 窗口的半对角线约 2.12 m，原 8 m 清除范围在窗口外浪费算力）；
- `failure_tolerance: 0.5 → 1.0`、DWB `transform_tolerance: 0.3 → 0.5`。

> 实车验证后，R5–R8（控制 20 Hz、转弯/加减速参数、inflation 0.40）和
> 雷达 12 Hz 也已启用，详见下方「实车验证」。

### 实车验证（2026-11-05，全部参数启用）

启用配置：`controller 20 Hz / smoother 25 Hz / DWB 角加速度 0.9、线加速度
0.35 / 局部 update 12 Hz、raytrace 3 m、inflation 0.40 / 雷达 12 Hz`。

bag：`/home/pi/rosbags/live_allparams_20261105_071722`
（131 s；5 次 `/goal_pose`，其中 1 次被新目标 preempt，实际完成 4 个目标）。

| 指标 | 基线 bag（10 Hz 雷达/旧参数） | 全参数 bag |
|---|---|---|
| 目标成功 / 失败 | 3 / 1 | **4 / 0** |
| FollowPath 失败 | 4 | **0** |
| 雷达周期中位数 / 最大值 | 0.0997 / 0.1049 s | **0.0827 / 0.0877 s** |
| 雷达频率 | 10.0 Hz | **12.096 Hz** |
| 局部代价地图更新 / 发布 | 10.0 / 6.0 Hz | **12.0 / 6.0 Hz** |
| 平滑器输出 | 18.3 Hz | **23.8 Hz** |
| 局部规划输出 | 13.6 Hz | **18.8 Hz** |
| 指令平均加速度（线/角） | 0.034 / 0.281 | 0.040 / 0.325 |
| odom 路径长度 | 33.66 m | 13.47 m |

结论：

- 本次 4/4 目标成功、0 次 `FollowPath` 失败；但路线比基线短
  （13.47 m vs 33.66 m），不是严格 A/B，只能说明该参数组合可用；
- 雷达 12 Hz 稳定，帧间隔最大 0.0877 s，没有掉帧；
- 局部代价地图更新 12 Hz、发布 6 Hz（发布仍是可视化限速）；
- 平滑器 23.8 Hz、局部规划 18.8 Hz，指令无前进反向；
- 一个目标耗时 47.9 s（成功），已定位为该目标末段**原地大角度旋转打滑**，
  见下节。

因此 R5–R8 与雷达 12 Hz **全部保留**在
`nav2_params_prior.yaml` / `TminiPlus.yaml`。后续仍需更长路线、多次复跑
验证，重点观察 `inflation_radius=0.40` 的贴障余量和 47.9 s 目标。

### 大角度原地旋转打滑（2026-11-05）

`live_allparams_20261105_071722` 中 47.9 s 目标的末段：

| 量 | 数值 |
|---|---:|
| 指令角速度 | 0.55 rad/s |
| 轮速反推转角 | 238.1° |
| IMU 陀螺积分 | 78.6° |
| rf2o 激光里程计 | 80.3° |
| EKF `/odom` | 80.0° |

轮速反馈达到了目标（左右轮约 ±0.9 rad/s），但陀螺和激光一致表明车体只转了
约 **1/3**。所以不是 DWB 在犹豫，而是**原地旋转时驱动轮打滑**：脚轮/地面
阻力超过轮胎附着，轮子空转，控制器只能反复补转，表现为“转很久、在犹豫”。

已做的修复：

- 固件 `APP_DEFAULT_CURRENT_A: 2.4 → 4.0 A`（电机最大 11 A），提高克服
  脚轮阻力的扭矩；已 build + flash + 台架 ARM 测试 PASS；
- DWB `acc_lim_theta: 0.9 → 0.6`、`decel_lim_theta: -1.1 → -0.8`；
  平滑器角加速度同步回到 0.6 / -0.8，避免起转瞬间扭矩尖峰加剧打滑；
- `RotateToGoal.scale / slowing_factor` 恢复为 20.0 / 5.0；
- `max_vel_theta` 曾降到 0.40，探针证明打滑与速度无关后**恢复 0.55**；
- `chassis_bridge` 新增 `/chassis/wheel_current`（Float32MultiArray，
  `[left, right]` A），下次录包请把它加入记录列表。

`tools/yaw_slip_probe.py` 实测（4 A 固件，原地纯旋转）：

| wz 指令 | 目标轮速 | 实际轮速 | 轮速反推 | 陀螺实际 | 实际/轮速 | 左/右电流 |
|---:|---:|---:|---:|---:|---:|---:|
| 0.15 | — | ±0.27 | -0.151 | -0.098 | 0.65 | 0.11 / -0.13 A |
| 0.25 | — | ±0.45 | -0.253 | -0.181 | 0.71 | 0.08 / -0.14 A |
| 0.35 | ±0.62 | ±0.62 | -0.349 | -0.264 | 0.76 | 0.11 / -0.12 A |
| 0.45 | ±0.80 | ±0.80 | -0.448 | -0.337 | 0.75 | 0.05 / -0.14 A |
| 0.55 | ±0.98 | ±0.98 | -0.551 | -0.431 | 0.78 | 0.08 / -0.11 A |

结论：

- **轮速在所有转速下都跟得上目标**（0.55 时 0.98 = 目标 0.98）；
- **实际/轮速稳定在 0.75–0.78，与速度无关**，即约 24% 的机械滑移；
- **4 A 电流上限不是瓶颈**：各转速电流都只有约 0.1 A，没有触发限流；
- 压低角速度不会改善打滑比例，只会让实际转向更慢，因此
  `max_vel_theta` 保持 **0.55**（之前误加的 0.40 已撤回）；
- bag 里 47.9 s 目标末段实际/轮速约 **0.33**，明显差于探针，说明当时
  有额外的脚轮卡滞/地面附着/贴障，而不是速度或电流设置问题。

保留的改动：更缓的角加速度（0.6 / -0.8）和 `RotateToGoal` 提前减速
（20 / 5.0），用于减少起转/停转时的扭矩尖峰；4 A 电流上限保留作余量，
但不是本次问题的原因。下一步应检查 bag 中那次大角度回头的位置：
脚轮是否自由转向、轮胎是否打滑、是否贴墙/压到线缆、地面是否过滑。

机械排查（软件无法替代）：脚轮是否自由转向、轮胎是否磨损/打滑、驱动轮
承重是否不足（重心/配重）、地面是否过滑。若这些正常，再考虑把电流上限
提高到 6 A 做对比（当前 4 A 未触发，不建议盲目再升）。

### 待解决

- 基线 bag 的 4 次 `FollowPath` 失败都是 progress checker（首个目标约
  39 s 未移动 0.15 m）触发；本次长参数 bag 未复现，继续用更长路线确认
  是目标/环境还是底盘卡滞。
- 最终以实车复跑验证全部参数和 `+80..+110°` 激光掩码。

回放数据在 `/home/pi/rosbags/nav2_replay/`：
`R1_baseline/`、`R2_robust/`、`rounds_20261105_0630/`、
`rounds_20261105_0700/`（每轮含 `metrics.json`、`params.yaml`、日志）。

> `run_nav2_replay.sh` 用 launch 日志里的 `Managed nodes are active` 判断激活，
> 不调用 `ros2 lifecycle get`：ROS 2 daemon 无响应时 CLI 会无限挂起（实测踩过）。

## 启动顺序

主机原生栈（推荐）：

1. `bash scripts/start_host_stack.sh [--motor]`
2. 该脚本依次启动导航（`ros_jazzy`）、AI 语音（`ai_host`）、手部检测

旧的 Docker + MQTT 栈（`scripts/start_stack.sh`）仅作回退，不再用于语音。

## DDS 隔离

| 栈 | ROS_DOMAIN_ID |
|---|---|
| 导航（主机 ros_jazzy） | 1 |
| AI 语音（主机 ai_host） | 1 |
| 旧 AI 容器（Humble） | 2 |

主机栈同域直发 `/goal_pose`；只有旧的容器栈需要 MQTT 桥接。

## 常见问题

| 现象 | 排查 |
|---|---|
| 容器看不到串口 | 启动参数是否包含 `--device`；`/dev/ydlidar` 软链接是否在宿主创建 |
| `No memory limit support` | 内核禁用了 memory cgroup；earlyoom + `ulimit -v` 兜底 |
| 云端 LLM 无响应 | `.env` 中的 Key 是否已轮换并生效 |
| 视觉不可用 | `vision_platform` 是否为 `tongyi`；云端网络是否可达 |
| 旧路径脚本失效 | `/home/pi/yahboom_ws` 是指向 `ai_ws` 的符号链接，确认链接存在 |

## 内存看门狗

- `robot-mem-watchdog.service` 常驻（systemd，开机自启）：
  - 每 2 秒读 `/proc/meminfo` 的 `MemAvailable`；
  - 连续 3 次（约 6 秒）低于 **800 MB** 时，终止整个机器人栈
    （导航 / AI / 深度 / 人像 / 手部）；
  - 先 SIGINT 让栈走正常清理（含 DISARM），5 秒后 SIGTERM，再 3 秒后 SIGKILL；
  - 触发后冷却 120 秒，避免重启瞬间重复触发；
  - **不杀** 桌面、SSH、VSCode、systemd，保留远程恢复能力。
- 查看状态：`systemctl status robot-mem-watchdog`、`journalctl -u robot-mem-watchdog -f`
- 临时调整阈值：改 `/etc/systemd/system/robot-mem-watchdog.service` 里的
  `MEM_THRESHOLD_MB`，再 `sudo systemctl daemon-reload && sudo systemctl restart robot-mem-watchdog`
- 测试（不发信号）：`MEM_DRY_RUN=1 MEM_THRESHOLD_MB=999999 MEM_CONFIRM_SAMPLES=1 bash scripts/mem_watchdog.sh`
- earlyoom 仍是最后兜底：已下调为可用内存 ≤5% / swap ≤20% 时动作，避免与看门狗抢跑
- 编译/重活：`ulimit -v 4194304` + `--parallel-workers 2`
- cgroup 恢复后：`docker info` 不再出现 `No memory limit support`，运行脚本会自动加内存上限


## 电脑低负载预览（2026-11-05）

Foxglove 图像由独立预览副本提供；本地原始深度仍为 320x240@30，人像识别输入和频率保留。
现有面板话题名不变：桥接节点内部将图像订阅重映射到 `/foxglove_preview/`。
默认彩色/标注图 240x180、JPEG 质量 40、最多 3 Hz；深度预览 160x120、16UC1、最多 3 Hz。
`FOXGLOVE_PREVIEW_WIDTH`、`FOXGLOVE_PREVIEW_FPS`、`FOXGLOVE_PREVIEW_QUALITY` 只调整电脑预览，
不调整相机硬件、模型输入或识别调度。详见 [FOXGLOVE_PREVIEW.md](FOXGLOVE_PREVIEW.md)。
