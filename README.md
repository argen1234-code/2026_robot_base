# 2026_robot_base

差速移动机器人底盘的 ROS 2 工作空间骨架，基于 **ROS 2 Jazzy**（Ubuntu 24.04）。

底盘的运动学正逆解和里程计由官方的 `diff_drive_controller` 负责，
本工作空间只提供模型描述、硬件接口和启动配置。

## 包结构

| 包 | 作用 |
|---|---|
| `robot_base_msgs` | 自定义消息与服务（`WheelSpeed`、`SetMotorEnable`） |
| `robot_base_description` | xacro 模型、RViz 配置、纯可视化 launch |
| `robot_base_driver` | ros2_control `SystemInterface` 硬件插件（当前为桩实现） |
| `robot_base_bringup` | 拉起整套栈的 launch 与控制器参数 |
| `ydlidar_ros2_driver` | 激光雷达驱动（从 [ros2_hunble_nav_jeston_orin_nano_super](https://github.com/argen1234-code/ros2_hunble_nav_jeston_orin_nano_super) 移植） |
| `rf2o_laser_odometry` | 激光里程计（同上仓库移植），当前 odom 的来源 |

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
cd ~/2026_robot_base
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
source ~/2026_robot_base/install/setup.bash
```

## 运行

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
| `/odom` | `nav_msgs/Odometry` | 里程计（激光里程计 rf2o），同时发布 `odom → base_footprint` TF |
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

本机已完成配置，规则在 `/etc/udev/rules.d/99-ydlidar.rules`：

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

改完 `/etc/udev/rules.d/99-ydlidar.rules` 后让规则生效：

```bash
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

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
ydlidar_ros2_driver -> /scan -> [5 级滤波] -> /scan_filtered -> RViz / slam_toolbox / rf2o
```

**RViz、slam_toolbox、rf2o 订阅的都是 `/scan_filtered`**；`/scan` 仍然存在（滤波链要读它）。

安装（不用教程里的 `rcm` 方式）：`laser_filters` 在标准 ROS 源里就有打包好的版本，
而教程给的 `curl -k https://www.ncnynl.com/rcm.sh | bash` 会**关闭 TLS 证书校验**
再把远程脚本直接交给 bash 执行，等于把机器交给那个站点且无法防篡改：

```bash
sudo apt install ros-jazzy-laser-filters
```

#### 当前滤波链

配置在 [config/laser_filters.yaml](src/robot_base_bringup/config/laser_filters.yaml)，
`filter1`~`filter5` **按编号顺序依次作用**，顺序会影响结果：

| # | 插件 | 作用 | 当前参数 |
|---|---|---|---|
| 1 | `LaserScanAngularBoundsFilter` | 角度裁剪 | `±3.1416`（**360°，等于不过滤**） |
| 2 | `LaserScanRangeFilter` | 距离截断 | `0.15~10.0 m`，超出置 `inf` |
| 3 | `LaserScanFootprintFilter` | 剔除车体自身 | `inscribed_radius: 0.15` |
| 4 | `ScanShadowsFilter` | **去阴影虚点** | `min/max_angle: 10/170`，`neighbors: 2` |
| 5 | `LaserScanSpeckleFilter` | 去孤立噪点 | `filter_type: 0`，`window: 2` |

顺序原则：先做范围性裁剪（1~3），再做邻域性分析（4~5）——
阴影和散斑都依赖相邻点关系，放在后面才不会把已经该删的点算进邻域统计。

**实测效果**（本机环境）：

```
/scan          平均有效 354.1 点/帧   -180°~180°
/scan_filtered 平均有效 328.4 点/帧   -180°~180°   滤掉 7.2%
```

360° 完整保留，只滤掉约 7% 的噪声/无效点，属温和设置。

#### ⚠️ 两个容易踩的点

**① 角度滤波只能"保留一段连续范围"，不能"砍掉中间一块保留其余"。**
`LaserScanAngularBoundsFilter` 的工作方式是**把数组裁剪到 `[lower_angle, upper_angle]`**。
而雷达本身扫描范围就是 −180°~+180°，所以设成 ±3.1416 时**一个点都不砍，是纯直通**。

要做"挖掉一个扇区"得用 `LaserScanSectorFilter`，参数是
`angle_min`/`angle_max`/`range_min`/`range_max`/`clear_inside`/`invert`
（缺任何一个都不生效）。实测它能把指定扇区的点设为 `range_max + 1`，
超过 `slam_toolbox` 的 `max_laser_range` 从而被忽略。

**② 角度参数单位是弧度，但 `ScanShadowsFilter` 的 `min_angle`/`max_angle` 是角度。**
同一个文件里两套单位，改参数时注意。

#### 想砍掉后方 90° 时

把 `filter1` 的 `lower_angle`/`upper_angle` 改成 `-2.3562`/`2.3562` 即可
（±135°，弧度）。换算：`弧度 = 角度 × 3.14159265 / 180`。

⚠️ 但注意这会在地图上留下**后方 90° 的空白**。本机底盘在雷达扫描面下方、
不在扫描范围内，所以通常没有东西需要砍 —— 保持 360° 更利于建图。

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

[slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html) 做 2D SLAM，
订阅 `/scan_filtered`，发布 `/map` 与 `map → odom` 的 TF。用 `slam:=false` 可关掉。

用的是 **lifelong（终身建图）模式**（节点 `lifelong_slam_toolbox_node`），
见下方「持续建图」。

### TF 树

```
map → odom → base_footprint → base_link → laser_frame
                  │              ├─ left_wheel_link
                  │              ├─ right_wheel_link
                  │              ├─ front_caster_link
                  │              └─ rear_caster_link
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

### 里程计：激光里程计（rf2o）

**当前 odom 的来源是激光雷达，不是轮子。**

```
/scan_filtered ─┬─→ rf2o_laser_odometry ─→ /odom + odom→base_footprint 的 TF（短时先验，会漂）
                └─→ slam_toolbox ────────→ map→odom 的 TF（扫描匹配修正）
```

#### 为什么这么做

`robot_base_driver.read()` **还是桩实现**（把速度命令回声成状态再积分），
轮式里程计完全没有反映真实运动。而雷达是真的 —— 所以激光里程计给出的才是真实运动。
**这带来一个实际好处：你现在推着车走就能建出真实地图，不用等电机接好。**

#### ⚠️ TF 归属：不能两边都发

`odom → base_footprint` 现在由 rf2o 发布，因此
[controllers.yaml](src/robot_base_bringup/config/controllers.yaml) 里
`diff_drive_controller` 的 **`enable_odom_tf` 必须为 `false`**。
两个节点同时发这条 TF 会让它有两个来源，树结构非法（RViz 报 `TF_REPEATED_DATA`）。

同理，launch 里**去掉了** `'/diff_drive_controller/odom' → '/odom'` 的重映射，
`/odom` 现在只有 rf2o 一个发布者。轮式里程计仍可在 `/diff_drive_controller/odom` 单独查看。

#### 实测

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

### 持续建图（lifelong 模式）

slam_toolbox 有两种建图模式，本项目用的是后者：

| | `async` 模式 | **`lifelong` 模式（当前）** |
|---|---|---|
| 节点 | `async_slam_toolbox_node` | `lifelong_slam_toolbox_node` |
| 位姿图 | 只增不减 | **会主动淘汰过时节点** |
| 环境变化 | 新旧两套墙都留在图上，地图糊掉 | 逐步替换，地图持续演化 |

lifelong 的核心参数（都在 [config/slam_toolbox.yaml](src/robot_base_bringup/config/slam_toolbox.yaml)）：

| 参数 | 当前值 | 作用 |
|---|---|---|
| `lifelong_node_removal_score` | `0.04` | ⭐ 低于此分数的新观测会淘汰旧节点 —— 地图"演化"靠它 |
| `lifelong_iou_match` | `0.85` | 重叠度阈值 |
| `lifelong_minimum_score` | `0.1` | 匹配分数下限，低于此不认为是重访 |
| `lifelong_nearby_penalty` | `0.001` | 邻近区域的惩罚项 |

> ⚠️ 两个模式的参数文件**不通用**：`lifelong_*` 这几项 `async` 节点没有声明，
> 直接复用会报未声明参数错误。换模式要连参数文件一起换。

#### 跨重启继续建图

lifelong 模式默认也**不持久化** —— 重启从头开始。要让地图跨重启累积，
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

之后启动会加载已有位姿图并在其上继续演化。

#### 换回单次建图

把 [robot_base.launch.py](src/robot_base_bringup/launch/robot_base.launch.py) 里
`executable` 改成 `async_slam_toolbox_node`，并把参数文件里的 `lifelong_*` 那几项删掉。

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

### 保存地图

```bash
# 运行时保存（同时存 pgm/yaml 与位姿图）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/argen/map}"
# 或用 nav2 的 map_saver_cli
ros2 run nav2_map_server map_saver_cli -f /home/argen/map
```

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

等真机编码器接好后，轮式里程计在差速底盘上通常更可靠
（激光里程计在长走廊等特征稀疏环境会失效），届时按上方「接真机后换回轮式里程计」切换。

### RViz 视图

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

- IMU 接入与 `robot_localization` EKF 融合（雷达已接入，见上）
- Nav2 导航（AMCL / costmap / planner；SLAM 建图已可用，见上）
- Gazebo 仿真（需另装 `ros-jazzy-ros-gz`）
- 真机硬件（`robot_base_driver` 的 read/write 仍是桩实现，详见「接真机」）
