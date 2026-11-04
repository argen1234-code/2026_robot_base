# 2026_robot_base

两轮差速底盘的 ROS 2 Jazzy（Ubuntu 24.04）工作空间。真机室内导航通过 MCU USB CDC 模式 3 接收前进和旋转速度指令；rf2o 激光、轮速和 IMU 陀螺经 EKF 松耦合融合成 `/odom`。该方案仍需实车速度和方向标定。

`diff_drive_controller`/`robot_base_driver` 仍是桩实现，不读取真实编码器；默认启动用 `joint_state_publisher` 提供关节状态，实际运动走 `chassis_bridge.py` + MCU。新 URDF 里已经包含 `<ros2_control>` 段，但默认不启动 controller_manager。不要用命令回声里程计当闭环反馈。

## 包结构

| 包 | 作用 |
|---|---|
| `robot_base_msgs` | 自定义消息与服务（`WheelSpeed`、`SetMotorEnable`） |
| `robot_base_description` | xacro 模型、RViz 配置、纯可视化 launch |
| `robot_base_driver` | ros2_control `SystemInterface` 硬件插件（当前为桩实现） |
| `robot_base_bringup` | 拉起整套栈的 launch 与控制器参数 |
| `ydlidar_ros2_driver` | 激光雷达驱动（从 [ros2_hunble_nav_jeston_orin_nano_super](https://github.com/argen1234-code/ros2_hunble_nav_jeston_orin_nano_super) 移植） |
| `rf2o_laser_odometry` | 激光里程计原始输出 `/odom_rf2o`，经 ZUPT 后进入 EKF |

依赖方向：`msgs` → `driver` → `bringup`；`description` 独立。

## 依赖

除 ROS 2 Jazzy 基础安装外，还需要：

```bash
sudo apt install python3-colcon-common-extensions \
                 ros-jazzy-controller-manager \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-ros2controlcli
```

- `controller_manager`：提供 `ros2_control_node` 和 `spawner`，是 `robot_base_bringup` 的运行时依赖
- `joint_state_publisher_gui`：供 `view_robot.launch.py` 的关节滑条使用
- `ros2controlcli`：提供 `ros2 control` 命令，用于 `list_controllers` 等排查手段

## 编译

```bash
cd ~/2026_robot_base/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`--symlink-install` 让 launch / yaml / xacro 改动后不必重新编译（C++ 改动仍需重编）。

单独重编某个包：

```bash
colcon build --packages-select robot_base_driver
```

## 环境

每个新终端都要 source：

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026_robot_base/robot_ros2_ws/install/setup.bash
```

## 运行

### 单雷达建图与 Nav2 导航（真机）

此工作区的真机运动模型使用底盘 MCU 的 ROS 室内模式；Nav2 控制器被限制为前进和原地旋转。`/odom` 由 EKF 融合 rf2o、轮速和 IMU 陀螺产生，建图时由 SLAM 发布 `map -> odom`；导航时关闭 SLAM，由 AMCL 接管该 TF。两种模式不要同时启动。

两轮 CAD 模型（2026-11-04）的整车包络为长 0.291 m、宽 0.236 m（含轮宽），
`base_footprint` 在两轮接地点中点、地面高度。URDF 的半透明圆盘
`navigation_footprint` 圆心在 `x=+0.1005`；Nav2 本地/全局 costmap 多边形
footprint 为
`[[-0.0450,-0.1180],[-0.0450,0.1180],[0.2460,0.1180],[0.2460,-0.1180]]`、
`footprint_padding: 0.0`，膨胀半径 0.50 m。安装额外外凸设备或换轮子时须
重新测量并同步修改 URDF、`nav2_params.yaml`、`controllers.yaml` 与固件常数。

### 统一启动入口（推荐）

整个工程只需要一个入口，用 `slam` 切换模式：

```bash
# 建图模式：默认加载 ~/maps 最新 posegraph 续建；没有则从零建图
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=true motor_enable:=true visualization:=foxglove

# 先验地图模式：slam_toolbox localization + Nav2
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:=$HOME/maps/robot_map \
  motor_enable:=true visualization:=foxglove
```

- `map` 是 **posegraph 前缀**（不带 `.posegraph`/`.yaml`）；
- `slam:=true`：默认用 `~/maps` 下最新 `*.posegraph` 续建；没有则从零建图；
  `map:=<prefix>` 指定特定地图，`map:=none` 强制从零建图；
- `slam:=false`：默认用 `~/maps` 最新 posegraph；没有会在启动节点前报错；
- `slam:=false` 使用 **slam_toolbox localization**，不再使用 AMCL/map_server；
  初始位姿由 `map_start_pose`（默认 `[0,0,0]`）或 `map_start_at_dock` 决定；
- `bringup.launch.py` 只是组合 `mapping_navigation.launch.py` 和
  `navigation.launch.py`，两个底层入口仍可单独调试。

### Nav2 鲁棒性策略

不是单纯调 Nav2 参数：

- `goal_sanitizer` 接管 `/goal_pose`，经 `/goal_pose_safe` 交给 `bt_navigator`；
  - 终点在致命格：1 m 内找最近自由格并替换；找不到则拒绝；
  - 起点在致命格：清 local/global costmap，再由 BT 的 `BackUp`/`Spin` 恢复；
  - 目标 frame 建议用 `map`；若为 `base_link`，stamp TF 失败会回退最新 TF；
- 使用 Nav2 默认 BT（已包含清图、spin、wait、backup 恢复）；
  参考仓库的旧版自定义 BT 与当前 Jazzy `bt_navigator` 节点 ID 不兼容，
  未在运行时启用；`behavior_trees/` 中文件仅作参考；
- 观测：`/goal_pose_safe`、`/goal_sanitizer_status`。

### 底层入口（调试用）

先建图：

```bash
ros2 launch robot_base_bringup mapping.launch.py
```

无已保存地图、需要设置目标并观察规划路径时，使用建图与 Nav2 联合入口（默认不驱动电机）：

```bash
ros2 launch robot_base_bringup mapping_navigation.launch.py \
  motor_enable:=true visualization:=foxglove
```

`visualization` 默认 `foxglove`（本机 RViz2 在 Ogre 初始化时会段错误），
浏览器打开 `https://studio.foxglove.dev` 连接 `ws://<树莓派IP>:8765`。
内存紧张时用 `visualization:=none`，另开电脑用 Foxglove/RViz 查看 ROS 话题。

等待雷达产生 `/map` 且 Nav2 的 `/planner_server`、`/bt_navigator` 进入 active 后，在 RViz 使用 **Nav2 Goal** 指定目标；红色为 `/plan` 全局路径，蓝色为 `/local_plan` 局部路径。没有地图覆盖的区域无法保证规划成功。该入口只启动一份 SLAM，不启动 AMCL；不要同时运行 `navigation.launch.py`。本入口无地图文件要求，电机默认关闭，因此即使规划成功也不会移动实车。

保存地图（建图栈运行中；同时生成 slam_toolbox posegraph 和 2D 图）：

```bash
bash tools/save_slam_map.sh ~/maps/robot_map
```

生成：

```text
~/maps/robot_map.posegraph/.data   # slam_toolbox localization/续建使用
~/maps/robot_map.yaml/.pgm         # 仅供查看/其他工具
```

`map_saver_cli` 单独产生的 `.yaml/.pgm` **不能**用于 slam_toolbox localization。

载入先验地图并启动 Nav2（默认不 ARM）：

```bash
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:=$HOME/maps/robot_map \
  motor_enable:=false visualization:=foxglove
```

确认扫描、定位和路径正常后，显式启用底盘输出：

```bash
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:=$HOME/maps/robot_map \
  motor_enable:=true visualization:=foxglove
```

`chassis_bridge` 走 STM32 原生 USB CDC（`/dev/stm32`，0483:5740）：下行是 `AA 55` 12 字节速度帧（模式 3、速度 float、XOR），以及 `BB 55` 4 字节 ARM/DISARM 服务帧；上行是 `CC 55` 104 字节遥测（50 Hz，轮速/目标/电流、IMU 的 roll/pitch/yaw/陀螺/加速度、ARM/超时/故障标志）。ROS 前进速度会按固件约定转换为负 Vx。先验地图启动时桥接限速为 0.24 m/s、0.55 rad/s，建图启动时为 0.20 m/s、0.45 rad/s；命令超时 0.25 秒即周期发送零速，短于 MCU 的 0.5 秒离线保护。STM32 与雷达的稳定设备名由 `src/robot_base_bringup/udev/99-robot-base.rules` 提供（`/dev/stm32` 与 `/dev/ydlidar`）：STM32 按 VID:PID `0483:5740` 绑定，雷达按物理口 `KERNELS=="3-2"` 绑定，不要把二者配成同一个名字。固件室内模式不支持倒车和横移，桥接会拒绝倒车命令。首次上电请架空车轮并备好物理急停；完成实际方向、轮廓尺寸和制动距离标定前，不要无人值守运行。

首次部署 udev 规则（规则模板位于 `src/robot_base_bringup/udev/99-robot-base.rules`）：

```bash
sudo install -m 0644 install/robot_base_bringup/share/robot_base_bringup/udev/99-robot-base.rules /etc/udev/rules.d/99-robot-base.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/stm32 /dev/ydlidar
```

只验证通信、不驱动车轮（发送 12 字节零速度帧）：

```bash
source install/setup.bash
ros2 run robot_base_bringup chassis_bridge.py --ros-args -p port:=/dev/stm32 -p enabled:=true
```

桥接启动日志出现 `STM32 connected` 即表示串口已打开；验证时不要发布非零 `/cmd_vel`。

微信小程序桥接按参考工程的 MQTT JSON/topic 协议实现，默认关闭。Broker 默认设为 `i6130f30.ala.cn-hangzhou.emqxsl.cn:8883`（MQTT over TLS）；TLS 使用系统 CA 信任链验证，无需把公共根证书内容作为密钥写入仓库。WebSocket TLS 端口 `8084` 不用于当前 ROS MQTT 客户端。先安装依赖 `sudo apt install python3-paho-mqtt`，配置 MQTT 用户名和密码，再分别显式启用导航、电机、微信桥接：

```bash
export WECHAT_MQTT_CLIENT_ID='robot_001_jetson'
export WECHAT_MQTT_USERNAME='your-username'
export WECHAT_MQTT_PASSWORD='your-password'
ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:=$HOME/maps/robot_map \
  motor_enable:=true wechat:=true
```

如需覆盖默认连接地址，可设置 `WECHAT_MQTT_BROKER` 和 `WECHAT_MQTT_PORT`。EMQX 管理 API Key 用于管理 REST API，不是 MQTT 登录凭据；请在 EMQX 控制台创建 MQTT 用户名/密码。API Key、用户名和密码均不要提交到 Git。

默认 topic 与参考小程序一致：订阅 `/k1ck5t83zdZ/test/user/get`，发布机器人状态、地图、路径和任务到 `robot`、`map`、`path`、`mission`。命令支持 `REMOTE`/`INDOOR`、方向键、`STOP`、`EMERGENCY`/`RESET_EMERGENCY` 和最多 30 个点的一次性 `INDOOR_MISSION_START`（坐标为 map 米，yaw 为弧度）；`LINE` 仅切换 MCU 室内模式，不启动循迹。地图以缩放灰度 PNG 的 base64 发布。急停/人工操作会撤销活动 Nav2 任务；遥控指令 0.25 秒失联自动归零。MQTT 断连后遥控速度因底盘超时归零，但既有 Nav2 任务仍继续运行。请勿在不可信网络开放云端遥控；固件物理急停仍是最终保护。

微信小程序代码位于本仓库的 `wechat-mini-program/` 目录。首次使用在小程序「状态」页填写专用 MQTT 客户端用户名和密码，保存后连接；凭据仅保存在微信本机存储中，不会提交到 Git。云端认证/授权配置参见仓库根目录 `deploy/emqx/README.md`。
当前 Jetson 侧未实现 GPS 任务，因此小程序默认只显示遥控和室内导航页面。

EMQX 最小权限模板和管理 API 示例位于仓库根目录 `deploy/emqx/`。建议 Jetson
与小程序使用两个不同的 MQTT 用户，避免小程序获得发布机器人状态的权限。

定位或电机不动作时，可在不启用电机的导航模式检查：

```bash
ros2 topic hz /scan_filtered
ros2 topic echo /odom --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /cmd_vel
ros2 lifecycle get /bt_navigator
```

`/cmd_vel` 是底盘输出前最后的碰撞监控速度指令。

### 1. 只看模型（不碰硬件）

```bash
ros2 launch robot_base_description view_robot.launch.py
```

会启动 `robot_state_publisher` + 关节滑条 + RViz，用于确认模型结构和关节轴方向。

### 2. 启动完整底盘栈

```bash
ros2 launch robot_base_bringup robot_base.launch.py
```

启动 `robot_state_publisher`、`ros2_control_node`（加载本仓库的硬件插件）、
`joint_state_broadcaster`、`diff_drive_controller`、`ydlidar_ros2_driver`、
`scan_to_scan_filter_chain`（角度过滤）、`slam_toolbox`（建图），以及 Foxglove Bridge。

可用参数：`visualization:=foxglove|rviz|none`、`lidar:=false`、`lidar_filter:=false`、
`slam:=false`、`lidar_port:=...`、`foxglove_port:=8765`。

### 3. 遥控与验证

```bash
# 另开终端
ros2 control list_controllers          # 两个控制器都应为 active
ros2 topic echo /odom --once           # 应有 header.frame_id: odom
ros2 lifecycle get /slam_toolbox       # 应为 active [3]
ros2 run tf2_tools view_frames         # 查看 TF 树，确认 map->odom->base_footprint 链完整

# ⚠️ Jazzy 的 diff_drive_controller 只接受 TwistStamped，
#    所以 teleop 必须打开 stamped，否则类型对不上、车不会动。
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

按键后 `/odom` 的位姿应随之变化。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 速度指令输入（**必须带 `header.stamp`**） |
| `/odom` | `nav_msgs/Odometry` | EKF 融合里程计（rf2o+ZUPT、轮速、IMU gyro z），同时发布唯一的 `odom → base_footprint` TF |
| `/odom_stabilized` | `nav_msgs/Odometry` | rf2o + ZUPT 的中间输出，供 EKF 使用，不发 TF |
| `/wheel/odom` | `nav_msgs/Odometry` | 轮速积分（有效轮径 0.041713 / 轮距 0.148），供 EKF 使用，不发 TF |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/scan` | `sensor_msgs/LaserScan` | 雷达**原始**扫描，10 Hz，`frame_id: laser_frame` |
| `/scan_filtered` | `sensor_msgs/LaserScan` | 角度过滤**后**的扫描 —— slam_toolbox 用这个 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 建出的栅格地图（`slam:=false` 时无） |
| `/point_cloud` | `sensor_msgs/PointCloud` | 同上数据的点云形式 |

`/cmd_vel` 与 `/odom` 是由 `robot_base_bringup/launch/robot_base.launch.py`
里的重映射从控制器的私有话题桥接出来的。

⚠️ Jazzy 起控制器只订阅 `~/cmd_vel` 且类型固定为 `TwistStamped`
（Humble 的 `use_stamped_vel` 参数与 `~/cmd_vel_unstamped` 话题均已移除），
因此往 `/cmd_vel` 发布的消息必须携带 `header.stamp`。

## 激光雷达

底盘上的雷达是 **YDLIDAR Tmini Plus**（USB 转串口 CP2102，`/dev/ydlidar`，230400 bps，
Model Code 151 / FW 1.2，实测 SN `2025110300090362`），由移植进来的
`ydlidar_ros2_driver` 包驱动，发布 `/scan`（默认 10 Hz，约 430 点/圈）。

⚠️ Tmini Plus 与 Tmini Pro 的参数不同：这台雷达不输出强度信息，必须用
`params/TminiPlus.yaml`（`intensity: false`、`intensity_bit: 0`）；照抄
`TminiPro.yaml`（`intensity: true`、`intensity_bit: 8`）会持续报
`Checksum error` 并反复切换强度位。SDK 默认开启强度位自动探测，启动瞬间
（电机起转/首帧不完整）仍可能打印几条校验和错误，稳定后 `/scan` 为 10 Hz。

`laser_frame` 是 URDF 里挂在 `base_link` 上的雷达 link，所以 TF 链
`odom → base_link → laser_frame` 由 `robot_state_publisher` 提供，
**不需要**额外的 `static_transform_publisher`。驱动的参数文件里 `frame_id`
也必须是 `laser_frame`，两边不一致 RViz 会报
`No transform from [laser_frame]`。

### 系统依赖：ydlidar_sdk

`ydlidar_ros2_driver` 依赖 C++ 库 `ydlidar_sdk`，它不在 ROS 源里，需从源码安装：

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install     # 装到 /usr/local，驱动靠 find_package(ydlidar_sdk) 找它
```

### 雷达端口绑定（udev）

Ubuntu 的 `/dev/ttyUSB*` 编号是按插入顺序**动态分配**的：插拔、重启、或同时接多个
USB 转串口设备时都会变。雷达参数文件里写死一个编号，换个口就找不到设备。

解决办法是用 udev 规则把雷达固定映射成一个稳定名字 `/dev/ydlidar`。
（方法参考 [USB端口绑定教程](https://gitee.com/gwmunan/ros2/wikis/%E5%AE%9E%E6%88%98%E6%95%99%E7%A8%8B/USB%E7%AB%AF%E5%8F%A3%E7%BB%91%E5%AE%9A)）

本机已完成配置。规则**模板在仓库里**：`src/robot_base_bringup/udev/99-robot-base.rules`
（那个文件同时管 STM32 和雷达两个设备，安装命令见本文档开头「首次部署 udev 规则」），
装好后生效于 `/etc/udev/rules.d/99-robot-base.rules`。雷达那一条是：

```
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="3-2", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="ydlidar"
```

各属性的来由（`udevadm info -a -n /dev/ttyUSB0` 得到）：

| 属性 | 值 | 说明 |
|---|---|---|
| `idVendor` | `10c4` | Silicon Labs |
| `idProduct` | `ea60` | CP2102 USB-UART 芯片 |
| `serial` | `0001` | ⚠️ **CP2102 的通用值，同型号设备全都一样，不能用来区分** |
| `KERNELS` | `3-2` | USB **物理口**（总线 3、端口 2）—— 区分同型号设备就靠它 |
| `devpath` | `2` | 兜底规则用；`KERNELS` 失效时仍能命中 |

**换 USB 物理口后 `KERNELS`/`devpath` 会变，规则会失效**，需要重新查并改这一行：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "looking at|ATTRS\{(devpath|idVendor|idProduct|serial)\}"
```

改完 `/etc/udev/rules.d/99-robot-base.rules` 后让规则生效（并顺手过一遍语法校验）：

```bash
sudo udevadm verify /etc/udev/rules.d/99-robot-base.rules
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
ls -l /dev/stm32 /dev/ydlidar
```

### STM32 端口绑定（udev）

STM32 烧录本仓库固件后会枚举为 `0483:5740` 的原生 USB CDC。**不要按 `ttyACM`
编号绑定**：`ttyACM0`/`ttyACM1` 会随枚举顺序变化（本机 DAPLink 的 VCP 就可能是
其中任意一个）。当前规则按 **VID:PID** 匹配：

```
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", GROUP="dialout", MODE="0660", SYMLINK+="stm32"
```

| 属性 | 值 | 说明 |
|---|---|---|
| `idVendor` / `idProduct` | `0483` / `5740` | ST Microelectronics 内置 USB CDC |
| `serial` | `337434543133`（本机） | ⚠️ **不要作为匹配条件**：重新枚举/旧固件下可能读成厂商串 `STMicroelectronics` |

> 历史：早先 `/dev/stm32` 按 USB 序列号绑定，重新枚举后序列号变成
> `STMicroelectronics`，规则不再命中，`chassis_bridge` 报
> `Cannot open MCU /dev/stm32: [Errno 2] No such file or directory`。
> 现在改按 VID:PID 匹配，换 USB 口也能命中。
>
> 调试器 DAPLink 的 VCP 单独命名为 `/dev/dap-vcp`（按物理口
> `KERNELS=="1-1.1"` + `faed:4873` 匹配），不会占用 `/dev/stm32`，
> 避免把“调试器串口”误当成“底盘 MCU 串口”。

**排查这类问题的三步**（不依赖 ROS）：

```bash
ls -l /dev/stm32 /dev/dap-vcp /dev/ydlidar
udevadm info -a -n /dev/ttyACM0 | grep -E "looking at|ATTRS\{(devpath|idVendor|idProduct|serial)\}"
udevadm trigger --subsystem-match=tty    # 重新触发一遍规则
```

> `ttyACM*` 编号会变，排查时先用 `lsusb` 或
> `udevadm info -q property -n /dev/ttyACM0 | grep ID_VENDOR_ID` 确认哪个是
> 0483:5740（STM32），哪个是 faed:4873（DAPLink）。

⚠️ 若旧部署里还留着 `/etc/udev/rules.d/99-stm32.rules`（同机重复副本，不在仓库里），
装本规则后请删掉它：`sudo rm -f /etc/udev/rules.d/99-stm32.rules`

验证（软链接名是稳定的，右侧 tty 编号可能互换）：

```bash
ls -l /dev/stm32 /dev/dap-vcp /dev/ydlidar
# 例如：
#   /dev/stm32   -> ttyACM0   (0483:5740)
#   /dev/dap-vcp -> ttyACM1   (faed:4873)
#   /dev/ydlidar -> ttyUSB0   (10c4:ea60)
```

> 教程示例用的是 `MODE:="777"`、`GROUP="users"`（任何用户可读写）。
> 这里改成 `GROUP="dialout"` + `MODE="0660"`：效果等价（当前用户已在 dialout 组），
> 但不会把串口开放给系统上所有用户。

### 串口权限

绑定规则里已经把组设成 `dialout`，当前用户必须是该组成员：

```bash
sudo usermod -aG dialout $USER    # 需要重新登录才生效
```

没权限（或没配 udev 规则、设备不在位）时雷达节点会报错退出，
但**不影响底盘其余部分**（可用 `lidar:=false` 关掉，或用 `lidar_port:=/dev/ttyUSB0` 临时绕开）。

### 激光滤波（laser_filters）

雷达原始数据 `/scan` 不做处理直接用会有噪声，所以中间加一层 `laser_filters` 滤波链：

```
ydlidar_ros2_driver -> /scan -> [车尾扇区遮罩] -> /scan_filtered -> RViz / slam_toolbox / rf2o
```

**RViz、slam_toolbox、rf2o 订阅的都是 `/scan_filtered`**；`/scan` 仍然存在（滤波链要读它）。

安装（不用教程里的 `rcm` 方式）：`laser_filters` 在标准 ROS 源里就有打包好的版本，
而教程给的 `curl -k https://www.ncnynl.com/rcm.sh | bash` 会**关闭 TLS 证书校验**
再把远程脚本直接交给 bash 执行，等于把机器交给那个站点且无法防篡改：

```bash
sudo apt install ros-jazzy-laser-filters
```

#### 当前滤波链：距离截断 + 车尾后向角度掩码

配置在 [config/laser_filters.yaml](src/robot_base_bringup/config/laser_filters.yaml)：

| # | 插件 | 作用 | 当前参数 |
|---|---|---|---|
| 1 | `LaserScanRangeFilter` | 去掉自回波和过远无效点 | `0.20 ~ 10.0 m`，越界替换为 `inf` |
| 2 | `LaserScanAngularBoundsFilterInPlace` | 掩掉车尾后向盲区 | raw `+80°..+110°` 置 `NaN` |

实测：

```
/scan          430 束/帧   -180.0°~180.0°
/scan_filtered 430 束/帧   0.20~10 m 距离截断；raw +80°..+110° 置 NaN
```

> 2026-11-05 `laser_frame` yaw 从 -90° 修正为 **+90°** 后，raw
> `laser_frame +90°` 经 TF 对应 `base_link` **车尾**（雷达后侧安装结构
> 的自回波，0.087 m 左右）；车头为 raw `-90°`。实测持续自回波在
> raw `+86.0°..+104.5°`；按 URDF 车体后向理论阴影，掩码取
> raw `+80°..+110°`（实测自回波 +86..+104.5，两侧留余量）。
> 2026-11-05 四轮离线回放中，`+80..+110` 的孤立连通域/unique occupied
> 更少，故最终采用该范围；`+66..+114` 作为车体阴影备选。

> 曾经试过串联 5 级滤波（角度 + 距离 + 车体 + 阴影 + 散斑），已全部移除，
> 原因见 [laser_filters.yaml](src/robot_base_bringup/config/laser_filters.yaml) 末尾的说明。
> 简要版：
> - **`LaserScanFootprintFilter` 没有可用默认值，会让整条链断掉** ——
>   不写 `inscribed_radius` 或写 `0.0` 时每帧报
>   `We need an index channel to be able to filter out the footprint`，
>   `/scan_filtered` **一帧都不输出**（不是不生效，是全断）。必须给正数半径。
> - 其余三个（range / shadows / speckle）的插件默认值本身接近空操作
>   （距离默认 0~100000 不截断；阴影默认 min=max=90 窗口极窄），
>   既然默认等于不干活就没必要留着。

#### 角度范围怎么改

当前滤波器屏蔽车尾扇区。雷达 `+Y`（绿色轴）指向车头，即 LaserScan 角度 `+90°`；
所以车尾为 `-90°`，左右各 45° 的屏蔽范围为 `-135°～-45°`。修改配置中的
`lower_angle`/`upper_angle`（弧度）即可调整遮罩边界：

| 遮罩范围 | `lower_angle` | `upper_angle` | 含义 |
|---|---|---|---|
| **车尾 90°（当前值）** | `-2.3562` | `-0.7854` | 车尾方向 ±45° |

换算公式：`弧度 = 角度 × 3.14159265 / 180`

角度遮罩会保留每帧原有的所有束位，只把车尾扇区的距离替换为 `NaN`，
表示该方向没有有效回波；扫描角度范围和束数不变。这里不能用 `range_max + 1`：
仓库中的 `rf2o` 将有限的超量程数值作为距离数据参与扫描配准，会导致位姿求解失败。

#### 其他可用插件

`laser_filters` 共提供近 20 个插件，除上表外还有：

| 类别 | 插件 |
|---|---|
| 角度/距离 | `LaserScanAngularBoundsFilterInPlace`（不缩短数组）、`LaserScanSectorFilter`（挖扇区）、`LaserScanIntensityFilter`（按强度）、`LaserScanMaskFilter`（屏蔽固定束号）、`LaserScanBinningFilter`（降采样） |
| 空间区域 | `LaserScanBoxFilter`（长方体）、`LaserScanPolygonFilter` / `StaticLaserScanPolygonFilter`（多边形） |
| 去噪/修数据 | `LaserScanMedianSpatialFilter` / `LaserMedianFilter`（中值平滑）、`ScanBlobFilter`（按连通块）、`InterpolationFilter`（插值补缺）、`LaserArrayFilter`（组合处理） |

完整列表见 `/opt/ros/jazzy/share/laser_filters/laser_filters_plugins.xml`，
加一条 `filter6:` 即可继续串联。

### 换型号 / 换串口

驱动包 `params/` 下自带各型号参数（X2 / X4 / G1 / G2 / TG / TminiPro /
TminiPlus ...）。本机实测是 **Tmini Plus**（Model Code 151，230400 bps，
默认 10 Hz、约 430 点/圈），launch 默认用 `params/TminiPlus.yaml`。
型号规格的扫描频率是 **6–12 Hz**；提高到 12 Hz 时每圈点数会降到约 358，
不适合需要更密点云的场景。

```bash
# 换成别的型号（型号名即 params/ 下的文件名，不带 .yaml）
ros2 launch robot_base_bringup robot_base.launch.py lidar_params_file:=/opt/ros/jazzy/share/ydlidar_ros2_driver/params/X4.yaml

# 换串口（默认已是 /dev/ydlidar）/ 关掉雷达
ros2 launch robot_base_bringup robot_base.launch.py lidar_port:=/dev/ttyUSB1
ros2 launch robot_base_bringup robot_base.launch.py lidar:=false
```

### 单独调试雷达

驱动包自带独立 launch，不加载本仓库的 URDF：

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py                 # 只起驱动
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py            # 带 RViz
ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_static_tf:=true  # 自己发 TF
```

`publish_static_tf` 默认 `false`：本仓库的 URDF 已经提供了 `base_link → laser_frame`，
再发一条同名的静态 TF 会让 RViz 报 `TF_REPEATED_DATA`。只有不加载 URDF 单独调试时才需要打开。

## 建图（slam_toolbox）

### 当前模型与运动学边界

`robot_base_description/urdf/robot_base.urdf.xacro` 已切换为 2026-11-04 接收的
**两轮差速 CAD 模型**（网格在 `meshes/`）：

- 几何轮半径 **0.041 m**；2026-11-05 标定有效滚动半径 **0.041713 m**；
  左右轮接地点中心 **y=±0.074 m**（有效轮距 **0.148 m**）；
- `base_footprint` 位于两轮接地点中点、地面高度，`base_link` 是 CAD 坐标系
  （`base_footprint → base_link = (0.075,0,0.103), yaw 0`）；
- `laser_frame` 的 yaw 为 **+90°**：raw 扫描 -90° 对应车头，+90° 对应车尾
  （自回波方向）；
- TF：`map → odom → base_footprint → base_link → link_laser → laser_frame`；
- 无麦克纳姆轮/横移，`diff_drive_controller` 与串口运动逻辑都是差速接口。

标定记录（2026-11-05）：

- 键盘 W/S/A/D 与实车一致；左转 yaw/gyro_z 增大、右转减小，IMU 轴符号
  与 ROS 一致；左右轮映射无需交换。
- IMU 静止零偏 `roll +2.005° / pitch -0.033° / yaw +218.260°` 已写入
  [config/imu_calibration.yaml](src/robot_base_bringup/config/imu_calibration.yaml)；
  但 JY901S 融合 yaw 在电机运行时会漂（13 s 约 97°，gyro_z 仅 -0.1°/s），
  **绝对 yaw 不可作为航向**，短时用 gyro_z/激光里程计。
- 速度：0.10 m/s × 12.88 s，编码器 4.998 圈 / 1.2876 m，卷尺 1.31 m，
  雷达正前方箱体 Δ1.319 m；有效轮径 0.041713 m，固件/控制器已同步并烧录。
  详见 [config/speed_calibration.yaml](src/robot_base_bringup/config/speed_calibration.yaml)。
- 工具：`tools/keyboard_cmd.py`、`tools/imu_calibrate.py`、
  `tools/run_speed_calibration.sh`。

`robot_state_publisher` 根据 `/robot_description` 发布 TF，Gazebo 只负责生成
模型和传感器，不是 TF 的发布者。旧的 `mecanum_car_model` 包保留作历史参考，
但已不再被导航模型依赖。

[slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html) 做 2D SLAM，
订阅 `/scan_filtered`，发布 `/map` 与 `map → odom` 的 TF。用 `slam:=false` 可关掉。

默认使用 **async 异步建图**（节点 `async_slam_toolbox_node`），
见下方「建图稳定性」。

### TF 树

```
map → odom → base_footprint → base_link → link_laser → laser_frame
                  │              ├─ left_wheel_link
                  │              └─ right_wheel_link
                  │
      ┌───────────┴────────────┬─────────────────────┐
      │                        │                     │
  slam_toolbox      rf2o_laser_odometry   robot_state_publisher
  （map→odom）        （odom→base_footprint）        （URDF）
```

⚠️ **`controllers.yaml` 里 `base_frame_id` 必须是 `base_footprint`，不能是 `base_link`。**

diff_drive_controller 会发布 `odom → <base_frame_id>`，而 URDF 里 robot_state_publisher
已经发布了 `base_footprint → base_link`。如果这里写 `base_link`，`base_link` 就会有
**两个父节点**（`odom` 和 `base_footprint`）—— TF 要求每帧只能有一个父节点，
结果是树结构非法，`base_footprint` 整条边被静默丢弃、从树上消失。

排查方法（教程里的那两条命令）：

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover   # 图形化，可刷新
ros2 run tf2_tools view_frames                      # 生成 PDF + frames.gv
```

### 里程计：EKF 松耦合（rf2o + 轮速 + IMU）

2026-11-05 起 `/odom` 由 `robot_localization` 的 `ekf_filter_node` 发布，
不再直接使用 rf2o：

```
/scan_filtered ─→ rf2o_laser_odometry ─→ /odom_rf2o
                                      └→ odom_stabilizer（轮速 ZUPT）→ /odom_stabilized
/chassis/wheel_speed ─→ wheel_odom ─→ /wheel/odom（vx, vyaw）
/chassis/imu ────────────────────────→ EKF imu0（只用 gyro z）
                                      ↓
                         ekf_filter_node → /odom + 唯一的 odom→base_footprint TF
/scan_filtered ─────────────────────→ slam_toolbox → map→odom
```

- `rf2o`：短时先验，特征稀疏环境会漂；
- `odom_stabilizer`：静止时用轮速冻结 rf2o，解决静态漂移；
- `wheel_odom`：用有效轮径 **0.041713 m**、轮距 **0.148 m** 积分 `vx/vyaw`；
- IMU：JY901S 融合 yaw 在电机运行时会漂，**只融合 gyro z**，不融合绝对 yaw；
- `ekf_filter_node`：`two_d_mode`，`world_frame=odom`，融合后发布 `/odom` 和 TF；
- `slam_toolbox` 仍只消费扫描和 TF，继续发布 `map→odom`，属于松耦合。

配置在 [config/ekf.yaml](src/robot_base_bringup/config/ekf.yaml)，
轮式里程计在 [scripts/wheel_odom.py](src/robot_base_bringup/scripts/wheel_odom.py)。

#### ⚠️ TF 归属

`odom → base_footprint` **只能由 `ekf_filter_node` 发布**：

- rf2o：`publish_tf=false`，只发 `/odom_rf2o`；
- `odom_stabilizer`：`publish_tf=false`，只发 `/odom_stabilized`；
- `wheel_odom`：只发 `/wheel/odom`，不发 TF；
- `diff_drive_controller` 的 `enable_odom_tf` 保持 `false`；
- EKF：`publish_tf=true`，输出 `/odom`（原生 `/odometry/filtered` 在 launch 中重映射）。

多个节点发同一条 TF 会报 `TF_REPEATED_DATA`，树结构非法。

#### 实测（2026-11-05）

- 静止 8 s：EKF `/odom` 位移约 **0.0000 m**，yaw 变化约 **0.0000°**；
- 1 m 目标测试：`/odom` 从原点到 **x≈1.04 m**，轮速/IMU 融合正常；
- EKF 输出约 **30 Hz**；Pi 同时跑 SLAM/Nav2/Foxglove 时偶发
  `Failed to meet update rate`，不是数据错误，可把 `ekf.yaml` 的
  `frequency` 从 30 降到 20 降低日志噪声。

历史：早期只用 rf2o + `odom_stabilizer` 的 ZUPT；现在保留 ZUPT 作为
EKF 的一个输入，并加入轮速/IMU 融合。

#### 源码移植说明

`rf2o_laser_odometry` 不在 ROS 源里，是从你另一个仓库
（`ros2_hunble_nav_jeston_orin_nano_super` 的 `handheld_mapping_ws/src/rf2o_laser_odometry`）
移植进来的，为适配 Jazzy 做了两处改动：

1. package.xml 移除 `cmake_modules`、`eigen`（ROS 1 时代的包，Jazzy 没有；
   且 CMakeLists 并未真正 find_package(cmake_modules)，eigen 已由 eigen3_cmake_module 取代），
   并补上 CMakeLists 实际用到却漏声明的 `nav_msgs`
2. `tf2_geometry_msgs/tf2_geometry_msgs.h` → `.hpp`（Jazzy 里 .h 已移除）

### 它是 LifecycleNode

`slam_toolbox` 的节点不是普通节点。直接 `ros2 run` 起的话，接口里**只有 lifecycle 样板**，
`/scan` 订阅和 `/map` 发布都要 configure + activate 之后才创建。本仓库的 launch 里已显式
发了这两个转换事件（`ChangeState` + `OnStateTransition`），所以开箱即用。

手动触发（调试用）：

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox          # 应为 active [3]
```

⚠️ 官方 launch 的 `use_sim_time` 默认是 `true`。**真机必须设 false**，
否则节点会一直等 `/clock` 而卡在未激活状态。本仓库已设为 `false`。

### 建图稳定性

默认使用 `async_slam_toolbox_node`，避免实验性 lifelong 模式的节点淘汰在
里程计质量有限时不断改写位姿图。rf2o 等待雷达 TF 后才初始化，并丢弃
重复时间戳及扫描几何参数异常的数据；雷达角度过滤范围保持不变。
没有轮速/IMU 时，rf2o 的扫描匹配仍可能在空旷、对称或动态环境漂移；
先降低车速，避开大面积玻璃和移动人群，闭环回到已走过的区域检查地图重影。
保存地图后用现有 AMCL 导航入口运行，不建议把实时建图视为长期稳定定位。

#### 跨重启继续建图

异步建图默认也**不持久化** —— 重启从头开始。要让地图跨重启累积，
先保存位姿图，再用 `map_file_name` 加载：

```bash
# 1) 建图跑完后保存（得到 map.posegraph 与 map.data）
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: /home/argen/map}"
```

```yaml
# 2) 在 config/slam_toolbox.yaml 里打开（注意与 map_start_at_dock 互斥）
map_file_name: /home/argen/map
map_start_pose: [0.0, 0.0, 0.0]
```

之后启动会加载已有位姿图并继续建图。地图完成后再保存栅格地图供 AMCL 使用。

### 参数

在 [config/slam_toolbox.yaml](src/robot_base_bringup/config/slam_toolbox.yaml)，改完重启即可：

| 参数 | 当前值 | 说明 |
|---|---|---|
| `scan_topic` | `/scan_filtered` | 用过滤后的数据 |
| `base_frame` | `base_footprint` | 必须与 `controllers.yaml` 的 `base_frame_id` 一致 |
| `resolution` | `0.05` | 栅格 5cm |
| `max_laser_range` | `12.0` | 与 Tmini Plus 量程一致 |
| `minimum_travel_distance` | `0.2` | 走多远插入一个位姿图节点（默认 0.5 对小车偏粗） |
| `map_update_interval` | `3.0` | `/map` 重发间隔（秒）；会话内地图就是按这个周期持续刷新的 |
| `use_scan_matching` | `true` | 关掉会退化成纯里程计推算，必须保持 true |
| `min_pass_through` | `1` | ⚠️ **已从默认值 2 改掉**，见下 |

### 保存地图

```bash
# 推荐：同时保存 posegraph 和 2D 图
bash tools/save_slam_map.sh /home/pi/maps/robot_map

# 等价于：
# ros2 service call /slam_toolbox/serialize_map \
#   slam_toolbox/srv/SerializePoseGraph "{filename: /home/pi/maps/robot_map}"
# ros2 run nav2_map_server map_saver_cli -f /home/pi/maps/robot_map
```

> 只有 `.posegraph/.data` 能被 slam_toolbox mapping/localization 加载；
> `.yaml/.pgm` 仅供查看或其他 2D 工具。统一入口默认取 `~/maps` 最新的
> `*.posegraph`。

### ⚠️ min_pass_through：默认值会让地图"看着只有一帧"

`slam_toolbox.yaml` 里的 `min_pass_through` 已从默认值 **2 改成 1**。

**含义**：一个栅格要被至少 N 条射线/扫描穿过，才标记为空闲。

**为什么改**：默认值 2 是为**连续运动**场景调的。车静止时位姿图只有一个节点、
只有一帧扫描，绝大多数栅格只被穿过 1 次 → 全被拒绝。

实测对比（两次都是"车静止 + 全新启动"，唯一变量是这个参数）：

| | `min_pass_through: 2`（默认） | `min_pass_through: 1` |
|---|---|---|
| 占用格 | **3** | **251** |
| 空闲格 | 766 | 3798 |
| 空闲区包围盒 | 2.05 × 4.60 m | **7.45 × 7.65 m** |

默认值下地图几乎是空的，RViz 里看着"只建了一帧、很小一块"，
要等车走得足够多、扫描互相叠加后地图才长出来。改成 1 后单帧即可建出完整地图。

**副作用**：单条射线即可判定空闲，对噪声更敏感（可能出现虚假空闲区）。
当前仅屏蔽车尾扇区，未启用去阴影/散斑滤波；
若发现地图上出现"幽灵通道"，改回 2。

### 怎么让地图长出来

slam_toolbox 只在移动超过 `minimum_travel_distance`（当前 0.2m）时才往位姿图里加节点。
所以**必须让车真的动起来**，地图才会增长。

现在 odom 来自激光里程计（rf2o），它反映的是**真实物理运动** ——
所以**用手推着车走**就能建图，不必等电机接好：

```bash
ros2 launch robot_base_bringup robot_base.launch.py     # 一个终端起栈
# 另一个终端看地图
ros2 topic echo /map --once | head -5
```

⚠️ 但**不要用 cmd_vel 假装让车动** —— 那样轮子（桩实现）会伪造里程计，
而 rf2o 的 TF 和它冲突，地图会变成沿虚构轨迹铺开的伪影。
（这也是为什么 `enable_odom_tf` 必须是 `false`：`diff_drive_controller` 的
`/cmd_vel` 仍然接在轮子上，但它不再影响 odom。）

真机编码器已通过 `wheel_odom` 接入 EKF（有效轮径 0.041713、轮距 0.148），
与 rf2o 和 IMU gyro z 松耦合；`diff_drive_controller` 仍是桩实现，
`enable_odom_tf` 保持 `false`，不参与 odom。

### 可视化视图（Foxglove / RViz）

```bash
# 默认：Foxglove Bridge 监听 8765，不启动本机 OpenGL
ros2 launch robot_base_bringup robot_base.launch.py visualization:=foxglove
# 单独起桥
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
# RViz（本机 RoboStack aarch64 当前会崩，修好后可用）
ros2 launch robot_base_bringup robot_base.launch.py visualization:=rviz
# 不启动任何可视化
ros2 launch robot_base_bringup robot_base.launch.py visualization:=none
```

浏览器打开 `https://studio.foxglove.dev` → Open connection → Foxglove WebSocket →
`ws://<树莓派IP>:8765`。推荐面板：3D（Fixed frame `map`，订阅 `/map`、
`/scan_filtered`、`/plan`）、Plot（`/chassis/wheel_speed`）、Raw Messages（`/chassis/imu`）。

**显示机器人模型**：URDF 不是普通 topic 显示项，要在 3D 面板设置里
「自定义图层 → 添加 URDF」：Source 选 **Topic** `/robot_description`
（或 Parameter `/robot_state_publisher.robot_description`），
Control mode=`Transforms`，Display mode=`Visual`。
`/local_costmap/published_footprint` 显示的是蓝色 footprint 多边形，不是车体模型。

> Foxglove 对 STL 默认按 **Y 轴向上** 加载；本车 CAD 是 Z 轴向上，若模型“躺着”，
> 在 3D 面板 → **场景（Scene）→ 网格上轴（Mesh up axis）** 改成 **Z-up**。
> 模型默认白色，和白色地图融为一体时可把 URDF 图层的颜色改成高对比色。

**在 Foxglove 里发导航目标（等同 RViz 的 2D Goal Pose）：**
Nav2 的 `bt_navigator` 直接订阅 `/goal_pose`（`geometry_msgs/PoseStamped`）。

1. 3D 面板右侧设置 → **Publish** 段，Type 选 **Pose**，Topic 填 `/goal_pose`；
2. 固定坐标系设为 `map`；
3. 点 3D 面板工具栏的 **Publish pose**（目标/箭头图标），在地图上点一下并拖出朝向。

也走不到的话，可在 **Publish** 面板手动发 `/goal_pose`，`header.frame_id: map`、
`pose.orientation.w: 1.0`；或命令行：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}"
```

[robot_base.rviz](src/robot_base_description/rviz/robot_base.rviz) 是建图视图，订阅：

| 显示 | 订阅的话题 | 说明 |
|---|---|---|
| Map | `/map`、`/map_updates` | SLAM 建出的栅格地图 |
| LaserScan | `/scan_filtered` | 角度过滤后的激光数据，渲染成点（绿色） |
| TF | `/tf`、`/tf_static` | 坐标系树 |
| Grid | 无 | 纯空间参照网格 |

已移除 RobotModel / Odometry（需要时右键 Add 自行加回）。
Fixed Frame 是 `map`；若 `slam:=false` 跑纯底盘，`map` 不存在，把 Fixed Frame 改成 `odom`。

⚠️ LaserScan 显示用的是 `sensor_msgs/LaserScan`，**不是 PointCloud**。
滤波链 `scan_to_scan_filter_chain` 输出的就是 LaserScan；驱动另发的 `/point_cloud` 是
**未过滤**的原始数据，且类型是已废弃的 `sensor_msgs/PointCloud`，滤波器不转发它。
QoS 用 Best Effort 是为了兼容性：滤波节点是 Reliable 发布（也能收），
雷达原始 `/scan` 是 Best Effort 发布，万一改回 `/scan` 不用改 QoS。

## Gazebo 仿真

```bash
ros2 launch robot_base_bringup gazebo.launch.py
ros2 launch robot_base_bringup gazebo.launch.py gui:=false    # 只跑服务器，不显示 Gazebo 界面
ros2 launch robot_base_bringup gazebo.launch.py visualization:=none
```

**这是与真机路径并存的第二条链路，不动现有代码。** 两条路径共用同一份 URDF，
靠 xacro 的 `use_gazebo` 参数切换硬件插件与传感器来源：

| | 真机路径 `robot_base.launch.py` | 仿真 `gazebo.launch.py` |
|---|---|---|
| 雷达 | ydlidar 驱动读串口 | **Gazebo 仿真**，经 `ros_gz_bridge` 桥接成 `/scan` |
| 轮子 | `robot_base_driver`（**桩实现，不会动**） | **Gazebo 物理引擎**（经 `gz_ros2_control`） |
| 时钟 | 系统时钟 | **仿真时钟** `use_sim_time:=true` |
| **`cmd_vel`** | **车不会动** | **车真的会跑** |

### ⚠️ 三个关键坑（都已处理，改的时候注意）

**① 不能再起 `ros2_control_node`。**
`gz_ros2_control` 插件会在 **Gazebo 进程内部**自己创建一个 `controller_manager`，
再起一个会撞车。本 launch 只用 spawner 往那个 controller_manager 里加载控制器。

**② 控制器必须延迟 12 秒再加载。**
Gazebo 启动阶段（加载世界 + 生成机器人 + 物理引擎初始化）很吃资源，
CM 的更新循环跟不上，会出现

```
[controller_manager]: Switch controller timed out after 5 seconds!
```

表现为 `joint_state_broadcaster` 加载成功但**激活失败** → `/joint_states` 无数据、
轮子不出现在 TF 里（车在 RViz 里是散的）。launch 里用 `TimerAction(period=12.0)`
延迟解决。**机器性能更差时可以调大。**

**③ RViz 的固定坐标系要用 `odom`，而且 `odom` 这条 TF 必须有人发。**
`robot_base.rviz` 默认 Fixed Frame 是 `map`，但仿真里没有 SLAM、没有 `map` 坐标系 ——
Fixed Frame 不存在时 RViz 什么都渲染不出来。launch 里已用 rviz2 的 `-f odom` 参数强制覆盖。

**⚠️ 但光改 Fixed Frame 不够 —— `odom -> base_footprint` 这条 TF 必须真的有人发布。**

`controllers.yaml` 里 `enable_odom_tf` 是 `false`，因为真机路径下这条 TF 由 rf2o 发布、
`diff_drive_controller` 刻意让出以免两个节点抢同一条。而 gazebo.launch.py **不启动 rf2o** ——
若不同时把 `enable_odom_tf` 改回 `true`，就【没有任何节点】发布 `odom` 坐标系，后果是：

- TF 树里根本没有 `odom` 帧（`view_frames` 只列出 `base_footprint -> base_link -> ...`）
- RViz 的 Fixed Frame 设成 `odom` 时什么都渲染不出来，小车完全不显示
- 依赖 `odom` 帧的下游（nav2 等）全部失效

因此 gazebo.launch.py 额外传了一份
[config/controllers_gazebo.yaml](src/robot_base_bringup/config/controllers_gazebo.yaml) 覆盖，
把 `diff_drive_controller` 的 `enable_odom_tf` 置回 `true`（spawner 的 `--param-file`
按顺序生效，后一个覆盖前一个）。

实测验证（发 `cmd_vel` 前进 0.3 m/s × 3 秒）：

| | `odom -> base_footprint` 的 x |
|---|---|
| 发之前 | 0.000 |
| 发之后 | **1.002** |

TF 确实随车移动在变 —— 这就是 RViz 里小车"会动"的充要条件。

### 实测结果（本机 Jetson Orin Nano）

```
/scan           9.6 Hz     仿真雷达（430 束、360°、量程 12m、带高斯噪声）
/scan_filtered  9.8 Hz     角度滤波后
/joint_states   49.6 Hz
/odom           48.3 Hz
diff_drive_controller / joint_state_broadcaster   均为 active
```

**发 `cmd_vel` 让车前进 0.3 m/s：**

| | x 坐标 |
|---|---|
| `/odom` | 1.008 m |
| **Gazebo 里的真实位姿** | **1.008 m** |

里程计与物理引擎真值逐位一致，说明轮子、控制器、里程计整条链路都对。

### 世界文件

[worlds/robot_base_world.sdf](src/robot_base_bringup/worlds/robot_base_world.sdf)：
一个 8m × 8m 的封闭房间（四面墙 + 两个障碍物）。

**为什么不做空世界**：空世界里雷达扫不到东西，建图会是一片空白，
看不出链路是否正常。有墙才能立刻验证。

世界文件里必须包含 **`gz-sim-sensors-system`** 插件，否则 `gpu_lidar` 不产生任何数据。

### 雷达在车上的位置

雷达装在**小车前方**：`robot_base.urdf.xacro` 里 `mount_x = 0.15`
（底盘 x 范围 ±0.20，0.15 表示靠近前缘并留出雷达半径余量）。改成正中就填 0。

### 尚未做

- 仿真里没启动 `rf2o` / `slam_toolbox` —— 仿真中轮式里程计是真实可靠的，
  直接用 `diff_drive_controller` 的 `/odom` 即可，不需要激光里程计。
  要仿真里也建图的话告诉我。
- 仿真里没有 IMU、相机。

## ⚠️ 改几何参数要同步改三处

两轮模型的 `wheel_separation` / `wheel_radius` 现在保持一致（0.148 / 0.041）：

1. `src/robot_base_description/urdf/robot_base.urdf.xacro`（CAD 模型/关节）
2. `src/robot_base_bringup/config/controllers.yaml`（差速里程计）
3. `stm32-firmware/RobStride05_H743/App/app_config.h`（MCU 轮速换算）

**三处不一致会导致 odom 尺度、轮速下发和实际行走距离全部对不上。**
新模型由 CAD 导出后生成，不再有 `wheel_radius:=` 这类 xacro 覆盖参数；
换轮子/改轴距时改 URDF 关节和上面两处常数。

## 接真机

`robot_base_driver/src/robot_base_system.cpp` 中的 `read()` 和 `write()`
是桩实现（把速度命令回声为状态），所以整套栈在没有硬件时也能跑通。

接真机时需要：

1. 实现 `on_configure()` 里的串口打开（代码中已标 `TODO(hardware)`），
2. 在 `read()` 里读编码器，写入 `hw_positions_` / `hw_velocities_`，
   再用 `set_state(position_states_[i], ...)` / `set_state(velocity_states_[i], ...)`
   推给框架——**不要**回读框架的 state 存储，它初值是 NaN；
3. 在 `write()` 里从命令接口句柄读下发值（rad/s），按你的电机协议打包：
   ```cpp
   double cmd = 0.0;
   get_command<double>(velocity_commands_[i], cmd, false);
   ```
4. 真机参数在 `robot_base.urdf.xacro` 的 `<ros2_control>` 段里
   （`serial_port=/dev/stm32`、`baud_rate=115200`）。注意实际导航运动走的是
   `chassis_bridge.py` + MCU USB CDC，`robot_base_driver` 仍是桩实现。

接口句柄在 `on_configure()`（状态）和 `on_activate()`（命令）里解析并缓存，
控制循环中只解引用裸指针，不做 map 查找，也不走 `wait_until_*` 阻塞路径。

## 尚未包含

- IMU 接入与 `robot_localization` EKF 融合（雷达已接入，见上）
- Nav2 导航（AMCL / costmap / planner；SLAM 建图已可用，见上）
- Gazebo 仿真（需另装 `ros-jazzy-ros-gz`）
- 真机硬件（`robot_base_driver` 的 read/write 仍是桩实现，详见「接真机」）
