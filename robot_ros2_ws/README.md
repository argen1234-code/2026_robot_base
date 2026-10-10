# 2026_robot_base

四轮底盘的 ROS 2 Jazzy（Ubuntu 24.04）工作空间。真机室内导航通过 MCU USB CDC 模式 3 接收前进和旋转速度指令；里程计由 robot_localization 的 EKF 融合「激光配准 rf2o + MCU 四轮轮速 + IMU 陀螺」产生。该方案仍需实车速度和方向标定。

仓库保留的 `diff_drive_controller`/`robot_base_driver` 是旧的双轮桩实现，不读取真实编码器，也不适用于当前四轮 CAD 模型；真机建图和导航不启用它。请勿把其命令回声里程计当作闭环反馈。

## 包结构

| 包 | 作用 |
|---|---|
| `robot_base_msgs` | 自定义消息与服务（`WheelSpeed`、`SetMotorEnable`） |
| `robot_base_description` | xacro 模型、RViz 配置、纯可视化 launch |
| `robot_base_driver` | ros2_control `SystemInterface` 硬件插件（当前为桩实现） |
| `robot_base_bringup` | 拉起整套栈的 launch 与控制器参数 |
| `ydlidar_ros2_driver` | 激光雷达驱动（从 [ros2_hunble_nav_jeston_orin_nano_super](https://github.com/argen1234-code/ros2_hunble_nav_jeston_orin_nano_super) 移植） |
| `rf2o_laser_odometry` | 激光里程计（同上仓库移植），现为 EKF 的一路输入（发布 `/odom_rf2o`）|
| `robot_localization` | EKF：融合轮速+IMU+rf2o，独占发布 `/odom` 与 `odom → base_footprint` |

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

此工作区的真机运动模型使用底盘 MCU 的 ROS 室内模式；Nav2 控制器被限制为前进和原地旋转。雷达扫描配准产生 `/odom`，建图时由 SLAM 发布 `map -> odom`；导航时关闭 SLAM，由 AMCL 接管该 TF。两种模式不要同时启动。

CAD STL 含四轮的最大水平外廓距 `base_footprint` 约 0.265 m。URDF 的半透明圆盘表示半径 0.30 m 的导航碰撞边界（含约 35 mm 安装余量），在 `view_robot.launch.py` 和建图/导航 RViz 中随车可见；导航圆盘在 Gazebo 中不作为物理接触体。Nav2 本地与全局 costmap 均使用 `robot_radius: 0.30`、`footprint_padding: 0.0`，膨胀半径 0.50 m，即从圆盘边缘向外 0.20 m 的渐变代价区；碰撞监控停止圈半径为 0.38 m。安装额外外凸设备时须重新测量并同步修改 URDF 与 Nav2 参数。

先建图：

```bash
ros2 launch robot_base_bringup mapping.launch.py
```

无已保存地图、需要在 RViz 设置目标并观察规划路径时，使用建图与 Nav2 联合入口（默认不驱动电机）：

```bash
ros2 launch robot_base_bringup mapping_navigation.launch.py
```

Jetson 内存紧张时可用 `visualization:=none` 关闭 RViz，另开电脑查看 ROS 话题。

等待雷达产生 `/map` 且 Nav2 的 `/planner_server`、`/bt_navigator` 进入 active 后，在 RViz 使用 **Nav2 Goal** 指定目标；红色为 `/plan` 全局路径，蓝色为 `/local_plan` 局部路径。没有地图覆盖的区域无法保证规划成功。该入口只启动一份 SLAM，不启动 AMCL；不要同时运行 `navigation.launch.py`。本入口无地图文件要求，电机默认关闭，因此即使规划成功也不会移动实车。

保存地图（文件名前缀，不带扩展名）：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/robot_map
```

载入地图并启动 Nav2，默认只观察 `/cmd_vel`，不连接/驱动 MCU：

```bash
ros2 launch robot_base_bringup navigation.launch.py map:=$HOME/maps/robot_map.yaml
```

确认扫描、定位和路径都正常后，架空驱动轮并显式启用底盘输出：

```bash
ros2 launch robot_base_bringup navigation.launch.py map:=$HOME/maps/robot_map.yaml motor_enable:=true mcu_port:=/dev/stm32
```

`chassis_bridge` 按 STM32 `develop` 固件 `bsp_usb.c` 的 12 字节 USB CDC 帧发送模式 3、速度 float 和 XOR 校验；ROS 前进速度会按固件约定转换为负 Vx。桥接限速为 0.20 m/s、0.45 rad/s，命令超时 0.25 秒即周期发送零速，短于 MCU 的 0.5 秒离线保护。STM32 与雷达的稳定设备名由 `src/robot_base_bringup/udev/99-robot-base.rules` 统一提供（`/dev/stm32` 与 `/dev/ydlidar`），两者都按 USB **物理口** `devpath` 绑定，不要把二者配成同一个名字。⚠️ 早先 STM32 是按 USB 序列号绑定的，但实测该序列号会失效，详见下方「STM32 端口绑定」。固件室内模式不支持倒车和横移，桥接会拒绝倒车命令。首次上电请架空车轮并备好物理急停；完成实际方向、轮廓尺寸和制动距离标定前，不要无人值守运行。

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
ros2 launch robot_base_bringup navigation.launch.py map:=$HOME/maps/robot_map.yaml motor_enable:=true wechat:=true
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
`scan_to_scan_filter_chain`（角度过滤）、`slam_toolbox`（建图），以及 RViz。

可用参数：`rviz:=false`、`lidar:=false`、`lidar_filter:=false`、`slam:=false`、`lidar_port:=...`。

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
| `/odom` | `nav_msgs/Odometry` | EKF 融合后的里程计（`/odometry/filtered` 重映射而来），同时发布 `odom → base_footprint` TF |
| `/odom_rf2o` | `nav_msgs/Odometry` | rf2o 的原始输出，EKF 的一路输入（不再发 TF）|
| `/wheel/odom` | `nav_msgs/Odometry` | 四轮计数算出的轮速里程计（静止时提供 ZUPT）|
| `/imu/data` | `sensor_msgs/Imu` | MCU 的陀螺角速度（加计仅诊断；姿态按约定标为未知）|
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

底盘上的雷达是 **YDLIDAR Tmini Pro**（USB 转串口 CP2102，`/dev/ydlidar`，230400 bps），
由移植进来的 `ydlidar_ros2_driver` 包驱动，发布 `/scan`（10 Hz）。

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
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="2.3", GROUP="dialout", MODE="0660", SYMLINK+="ydlidar"
```

各属性的来由（`udevadm info -a -n /dev/ttyUSB0` 得到）：

| 属性 | 值 | 说明 |
|---|---|---|
| `idVendor` | `10c4` | Silicon Labs |
| `idProduct` | `ea60` | CP2102 USB-UART 芯片 |
| `serial` | `0001` | ⚠️ **CP2102 的通用值，同型号设备全都一样，不能用来区分** |
| `devpath` | `2.3` | USB **物理口**编号 —— 区分同型号设备就靠它 |

**换 USB 物理口后 `devpath` 会变，规则会失效**，需要重新查并改这一行：

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

STM32 的 `/dev/stm32` 原先按 **USB 序列号** 绑定（序列号由固件 `usbd_desc.c` 的
`Get_SerialNum()` 从芯片 UID 派生，本意是芯片唯一、换口不改名）。**实测会失效：**

> 接调试器 / 复位后重新枚举时，主机读到的序列号会变成厂商串 `STMicroelectronics`，
> 按序列号匹配的规则于是不再命中，`/dev/stm32` 直接消失、`chassis_bridge` 开不了口：
> `Cannot open MCU /dev/stm32: [Errno 2] No such file or directory`，且每 5 秒重试一次。

所以现在**主规则改成按物理口 `devpath` 绑定**（与雷达同一写法），另外**保留一条按
序列号的规则作为兜底** —— udev 的多条规则是「或」关系，同时命中只是重复创建同一个
软链接、无害；这样把 STM32 插到别的口时，只要这次序列号读得出来，名字照样正确。

| 属性 | 值 | 说明 |
|---|---|---|
| `idVendor` / `idProduct` | `0483` / `5740` | ST Microelectronics 内置 USB CDC |
| `devpath` | `2.2` | USB **物理口**编号，与雷达的 `2.3` 同属集线器 `1-2` |
| `serial` | `3159396C3430` | ⚠️ 本应芯片唯一，但**会读成 `STMicroelectronics`**，只能当兜底 |

**排查这类问题的三步**（不依赖 ROS）：

```bash
ls -l /dev/stm32 /dev/ydlidar
udevadm info -a -n /dev/ttyACM0 | grep -E "looking at|ATTRS\{(devpath|idVendor|idProduct|serial)\}"
udevadm trigger --subsystem-match=tty    # 重新触发一遍规则
```

⚠️ 若旧部署里还留着 `/etc/udev/rules.d/99-stm32.rules`（同机重复副本，不在仓库里），
装本规则后请删掉它：`sudo rm -f /etc/udev/rules.d/99-stm32.rules`

验证（应看到 `ydlidar -> ttyUSB0`）：

```bash
ls -l /dev | grep -E "ttyUSB|ydlidar"
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

#### 当前滤波链：只保留角度滤波

配置在 [config/laser_filters.yaml](src/robot_base_bringup/config/laser_filters.yaml)：

| # | 插件 | 作用 | 当前参数 |
|---|---|---|---|
| 1 | `LaserScanAngularBoundsFilterInPlace` | 屏蔽车尾 90° | 雷达角 `-135°～-45°`；绿色 `laser_frame +Y` 为车头 |

实测：

```
/scan          430 束/帧   -180.0°~180.0°
/scan_filtered 430 束/帧   保留整圈角度元数据，车尾约 90° 的束标记为无效回波
```

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

驱动包 `params/` 下自带各型号参数（X2 / X4 / G1 / G2 / TG / TminiPro ...）。
本机实测是 TminiPro（230400 bps，10 Hz，实测有效距离 0.04~7.4 m），launch 默认用它。

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

当前 `robot_base_description/urdf/robot_base.urdf.xacro` 已完全使用仓库最新的
`mecanum_car_model` CAD 网格与四个麦克纳姆轮关节，并额外补齐
`base_footprint -> base_link -> laser_frame` 的 TF 链。`robot_state_publisher`
根据 `/robot_description` 发布 TF，RViz 的 `RobotModel` 订阅该话题显示模型；
Gazebo 只负责生成模型和传感器，不是 TF 的发布者。

当前仓库的串口硬件插件和 `diff_drive_controller` 仍然是“两轮差速”接口，
不能驱动四轮麦克纳姆底盘的横移/全向运动。因此默认建图启动使用
`joint_state_publisher` 发布关节状态，适合手推车或外部里程计建图；不要把
`/cmd_vel` 的差速控制结果当作麦克纳姆运动学。要实现真实四轮控制，需要另行
接入四轮控制器（或自定义麦克纳姆运动学控制器）及对应硬件接口。

[slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html) 做 2D SLAM，
订阅 `/scan_filtered`，发布 `/map` 与 `map → odom` 的 TF。用 `slam:=false` 可关掉。

默认使用 **async 异步建图**（节点 `async_slam_toolbox_node`），
见下方「建图稳定性」。

### TF 树

```
map → odom → base_footprint → base_link → laser_frame
                  │              ├─ left_wheel_link
                  │              ├─ right_wheel_link
                  │              ├─ front_caster_link
                  │              └─ rear_caster_link
                  │
      ┌───────────┴──────────────┬────────────────────────┐
      │                          │                        │
  slam_toolbox          ekf_filter_node          robot_state_publisher
  （map→odom）     （odom→base_footprint）★唯一         （URDF）
                     ▲        ▲        ▲
                     │        │        │
              /odom_rf2o  /wheel/odom  /imu/data
              （rf2o）   （chassis_bridge）（chassis_bridge）
```

⚠️ **`controllers.yaml` 里 `base_frame_id` 必须是 `base_footprint`，不能是 `base_link`。**

diff_drive_controller 会发布 `odom → <base_frame_id>`，而 URDF 里 robot_state_publisher
已经发布了 `base_footprint → base_link`。如果这里写 `base_link`，`base_link` 就会有
**两个父节点**（`odom` 和 `base_footprint`）—— TF 要求每帧只能有一个父节点，
结果是树结构非法，`base_footprint` 整条边被静默丢弃、从树上消失。

⚠️ **`odom → base_footprint` 这条边只能有一个发布者，现在是 `ekf_filter_node`。**
所以 rf2o 的 `publish_tf` 必须是 `False`、`diff_drive_controller` 的
`enable_odom_tf` 必须是 `false`。三个里任意两个同时发就树结构非法。

排查方法（教程里的那两条命令）：

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover   # 图形化，可刷新
ros2 run tf2_tools view_frames                      # 生成 PDF + frames.gv
```

### 里程计：EKF 融合（轮速 + IMU陀螺 + rf2o）

**odom 由 `robot_localization` 的 EKF 产生**，三路输入各有强弱，正好互补：

| 输入 | 话题 | 强在 | 弱在 |
|---|---|---|---|
| rf2o 激光配准 | `/odom_rf2o` | **平移**（米制、尺度正确）| 旋转（原地转/长廊退化，**漂移的来源**）|
| IMU 陀螺 `gyro_z` | `/imu/data` | **旋转** | 平移（测不到）|
| 四轮轮速 | `/wheel/odom` | 静止时给"**没动**"这个强约束 | 麦轮侧滑 → 有系统偏差 |
| （slam_toolbox）| TF `map→odom` | 绝对修正 | — |

#### 它解决什么问题：车静置久了位姿漂走

此前 odom 只有 rf2o 一路：它每帧都产生微小虚假运动并积分（实测静止 yaw 漂
**0.29 °/s**），而 slam_toolbox 的扫描处理被 `minimum_travel_*` 门控、**静止时不做
纠正** —— 于是位姿跟着 odom 漂。实测放一段时间就能漂到明显错误的位置。

修法是把"静止"这个信息喂给滤波器：**四轮计数为 0 时，`/wheel/odom` 以极小协方差
（1e-5）发布零速**（ZUPT，零速修正），EKF 的速度于是被强拉向 0、位姿被钉住。
运动时同一路改用大协方差（2.5e-3），让 rf2o 主导平移 —— **所以轮速没标定也不会
污染平移**（这是刻意设计，见 [ekf.yaml](src/robot_base_bringup/config/ekf.yaml) 注释）。

#### ⚠️ 陀螺零偏必须扣（EKF 没有零偏状态）

`robot_localization` 的 EKF 状态量里**没有陀螺零偏**，所以零偏得在发布前扣掉，
否则用陀螺换来的 yaw 改善会被它吃掉。`chassis_bridge` 里做了**静基座零偏估计**：
判据是"本模式指令为 0 + 四轮计数为 0 + |gyro−零偏| < 3 °/s"，先攒 50 个静止样本取
中位数，之后极慢泄漏自适应。实测零偏中位数约 **±0.07 °/s**（各次运行不同），
实时值发布在 `/imu/gyro_bias`（可读出后用 `gyro_bias:=` 硬写、去掉自动估计）。

#### ⚠️ 轮速标定（未做之前 odom 平移仍然可用，但轮速量级不准）

`motor_speed[i]` 是**每个控制周期**的带符号整数计数增量（FL,FR,RL,RR），而**控制
周期会变**（实测 35~71 ms），所以：

* **距离** 与周期无关：`distance_m = dist_counts / wheel_counts_per_mps`
* **速度** 需要周期：`v = c / (loop_period_ms/1000) / wheel_counts_per_mps`

`wheel_counts_per_mps` 的含义是"**每米行程累加多少个每周期计数**"，占位值 **12000**
（由固件命令尺度 `ROS_LINE_MAX_SPEED=120` 反推：120 counts/10ms × 100 个周期/秒），
但实测周期不是 10 ms，所以占位值预计偏差可达数倍 —— **必须实测标定**：

```bash
# 停掉栈（标定脚本要独占串口），前方留一段 ≥1 m 直线
python3 tools/twist_sign_probe.py --calibrate-wheel-scale
```
脚本会自己开环驱动小车直线前进（Enter 停止），用固件 `dist_counts` 的增量除以你输入
的实际米数，直接吐出 `wheel_counts_per_mps`，再用参数覆盖：

```bash
ros2 run robot_base_bringup chassis_bridge.py --ros-args \
  -p port:=/dev/stm32 -p enabled:=true -p wheel_counts_per_mps:=<标定值>
```

⚠️ 这是**直线**标定结果：原地转时麦轮侧滑会让编码器口径明显高估，所以**轮速的
`vyaw` 永远不能用于融合**（配置里已如此，`odom1_config` 只开 vx）。

#### 回退与降级（两者都要知道）

* **整体回退到"rf2o 独占 odom"（= v0.2.0 里程碑的行为）**：一个开关即可：

  ```bash
  ros2 launch robot_base_bringup mapping_navigation.launch.py motor_enable:=true ekf:=false
  ```

  `ekf` 开关同时决定 rf2o 与 EKF 的接线（由 launch 里的 OpaqueFunction 在运行期
  读取），两种模式下都**恰好一个** `odom→base_footprint` 发布者、且 `/odom` 的
  话题名不变。实测：`ekf:=false` 时只有 rf2o、`/odom` 由它发、该 TF 10 Hz；
  `ekf:=true` 时 rf2o 的 `publish_tf=False`、`odom_topic=/odom_rf2o`，TF 由 EKF 发。
  ⚠️ 早先的写法把 rf2o 的参数写死，`ekf:=false` 时**没有任何节点发这条 TF**，
  整条链断掉（表现为"膨胀层覆盖全图"）——已修正。
* ⚠️ **`motor_enable:=false` 时没有 `/wheel/odom` 与 `/imu/data`**：这两路来自
  `chassis_bridge`，而它只在 `motor_enable:=true` 时启动（且独占 `/dev/stm32`）。
  此时 EKF 只剩 rf2o 一路，**行为退化成与改动前相同（静止漂移会回归）**。
  这是预期，不是故障 —— 没有 MCU 链路就没有 MCU 数据。
* 单点回退：去掉陀螺 yaw → `ekf.yaml` 的 `imu0_config[11]=false`；去掉轮速融合 →
  `odom1_config` 全 `false`；停零偏扣除 → `-p subtract_gyro_bias:=false`。

#### 已知局限

* rf2o 的 twist 是**最近 5 帧的滑动平均**（约 0.5 s 相位滞后），所以 EKF 平移会带
  滞后；且停车后约 0.5 s 内 rf2o 的 vx 还没衰减到 0，会短暂顶住轮速的 ZUPT
  （协方差差 40 倍，零速仍占优，但可能有瞬态）。
* 本车在 ROS 室内模式下从不横移（固件 `Vy_set` 恒 0、`nav2` 的 `max_vel_y=0`），
  而 `vy` 没有任何可信观测，所以 EKF 的 `vy` 会一直≈0。
* 磁力计是坏的（`mag_*` 恒为 0，而 flags 的 `MAG_VALID` 位照置），JY901S 自己的
  `imu_yaw` 因此在漂（实测 0.2 °/s）—— 所以 `/imu/data` 按 `sensor_msgs/Imu` 约定
  把 `orientation_covariance[0]` 置 **−1**（"姿态未知"），下游不要把它当姿态用。

#### 实测（改动前，作对照基线）

车静止不动时，rf2o 的 `/odom` 会因雷达测量噪声缓慢漂移（实测约 10cm）：

| | 数值 |
|---|---|
| rf2o 的 `/odom` | `(0.024, -0.098)` |
| slam_toolbox 的 `map → odom` | `(-0.017, 0.098)` |

修正量几乎是漂移的**精确逆**，相乘后 `map → base_footprint` ≈ 单位矩阵，
车在 map 坐标系里被钉住不动。这就是这套架构的设计意图：
**激光里程计提供短时先验（会漂），SLAM 负责长期修正。**

实测地图 30 秒内只有 2 个版本、占用格 308→347 后收敛，没有被漂移糊掉
（对比：用桩里程计伪造运动时是 38→475、5 个版本快速变化，地图是伪影）。

#### 接真机后换回轮式里程计

差速底盘上轮式里程计通常更可靠 —— 激光里程计在**长走廊、空旷场地等特征稀疏环境会失效**，
而且只有 10Hz。等编码器接好：

1. [controllers.yaml](src/robot_base_bringup/config/controllers.yaml) 里 `enable_odom_tf` 改回 `true`
2. launch 里把 `rf2o_node` 那段注释掉
3. 恢复重映射 `'-r /diff_drive_controller/odom:=/odom'`

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
| `max_laser_range` | `12.0` | 与 Tmini Pro 量程一致 |
| `minimum_travel_distance` | `0.2` | 走多远插入一个位姿图节点（默认 0.5 对小车偏粗） |
| `map_update_interval` | `3.0` | `/map` 重发间隔（秒）；会话内地图就是按这个周期持续刷新的 |
| `use_scan_matching` | `true` | 关掉会退化成纯里程计推算，必须保持 true |
| `min_pass_through` | `1` | ⚠️ **已从默认值 2 改掉**，见下 |

### 保存地图

```bash
# 运行时保存（同时存 pgm/yaml 与位姿图）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/argen/map}"
# 或用 nav2 的 map_saver_cli
ros2 run nav2_map_server map_saver_cli -f /home/argen/map
```

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
而 EKF 的 TF 用的正是这份桩里程计（通过 rf2o/轮速），地图会变成沿虚构轨迹铺开的伪影。
（这也是为什么 `enable_odom_tf` 必须是 `false`：`diff_drive_controller` 的
`/cmd_vel` 仍然接在轮子上，但它不再影响 odom。）

等真机编码器接好后，轮式里程计在差速底盘上通常更可靠
（激光里程计在长走廊等特征稀疏环境会失效），届时按上方「接真机后换回轮式里程计」切换。

### RViz 视图

启动时可在 RViz 和 Foxglove 之间选择；默认仍为 RViz：

```bash
ros2 launch robot_base_bringup robot_base.launch.py visualization:=rviz
ros2 launch robot_base_bringup robot_base.launch.py visualization:=foxglove
ros2 launch robot_base_bringup robot_base.launch.py visualization:=none
```

Foxglove 模式需先安装桥接包：`sudo apt install ros-jazzy-foxglove-bridge`。
然后在 Foxglove 中添加
**Foxglove WebSocket** 连接，地址填写 `ws://localhost:8765`。
Bridge 默认监听 `0.0.0.0:8765`；从另一台电脑可直接连接 `ws://<机器人IP>:8765`，
或使用 SSH 端口转发后连接本机地址：
`ssh -L 8765:127.0.0.1:8765 <机器人用户名>@<机器人IP>`，并在电脑上的
Foxglove 连接 `ws://localhost:8765`。在 3D 面板中添加 `/robot_description`
机器人模型、`/tf` 与 `/tf_static` 坐标变换、`/scan_filtered` 激光扫描和
`/map` 地图；Foxglove 不会自动导入 RViz 的 `.rviz` 布局。

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
ros2 launch robot_base_bringup gazebo.launch.py visualization:=foxglove
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

`controllers.yaml` 里 `enable_odom_tf` 是 `false`，因为真机路径下这条 TF 由 EKF 发布、
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

## ⚠️ 改几何参数要同步改两个文件

`wheel_separation` 与 `wheel_radius` 在两个地方各有一份：

1. `src/robot_base_description/urdf/robot_base.common.xacro`（模型外观）
2. `src/robot_base_bringup/config/controllers.yaml`（里程计计算）

**两处不一致会导致 odom 的尺度和转向全部算错。** 改的时候务必一起改。

`robot_base.common.xacro` 里的参数都可以在命令行覆盖，不必改文件：

```bash
xacro src/robot_base_description/urdf/robot_base.urdf.xacro \
      wheel_radius:=0.06 wheel_separation:=0.40
```

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
4. 把 `robot_base.ros2_control.xacro` 里的 `serial_port` / `baud_rate` 改成真机值。

接口句柄在 `on_configure()`（状态）和 `on_activate()`（命令）里解析并缓存，
控制循环中只解引用裸指针，不做 map 查找，也不走 `wait_until_*` 阻塞路径。

## 尚未包含

- ~~IMU 接入与 `robot_localization` EKF 融合~~（已完成：见「里程计：EKF 融合」。
  ⚠️ 轮速标定还没做，`wheel_counts_per_mps` 是占位值 12000，见那一节的标定步骤）
- Nav2 导航（AMCL / costmap / planner；SLAM 建图已可用，见上）
- Gazebo 仿真（需另装 `ros-jazzy-ros-gz`）
- 真机硬件（`robot_base_driver` 的 read/write 仍是桩实现，详见「接真机」）
