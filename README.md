# 2026_robot_base

差速移动机器人底盘的 ROS 2 工作空间骨架，基于 **ROS 2 Humble**。

底盘的运动学正逆解和里程计由官方的 `diff_drive_controller` 负责，
本工作空间只提供模型描述、硬件接口和启动配置。

## 包结构

| 包 | 作用 |
|---|---|
| `robot_base_msgs` | 自定义消息与服务（`WheelSpeed`、`SetMotorEnable`） |
| `robot_base_description` | xacro 模型、RViz 配置、纯可视化 launch |
| `robot_base_driver` | ros2_control `SystemInterface` 硬件插件（当前为桩实现） |
| `robot_base_bringup` | 拉起整套栈的 launch 与控制器参数 |

依赖方向：`msgs` → `driver` → `bringup`；`description` 独立。

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
source /opt/ros/humble/setup.bash
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
`joint_state_broadcaster`、`diff_drive_controller`，以及 RViz。

### 3. 遥控与验证

```bash
# 另开终端
ros2 control list_controllers          # 两个控制器都应为 active
ros2 topic echo /odom --once           # 应有 header.frame_id: odom
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按键后 `/odom` 的位姿应随之变化。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令输入 |
| `/odom` | `nav_msgs/Odometry` | 里程计，同时发布 `odom → base_link` TF |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |

`/cmd_vel` 与 `/odom` 是由 `robot_base_bringup/launch/robot_base.launch.py`
里的重映射从控制器的私有话题桥接出来的。

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
2. 在 `read()` 里读取编码器数据，写入 `hw_positions_` / `hw_velocities_`，
3. 在 `write()` 里把 `hw_commands_`（rad/s）按你的电机协议打包下发，
4. 把 `robot_base.ros2_control.xacro` 里的 `serial_port` / `baud_rate` 改成真机值。

## 尚未包含

- 传感器接入（雷达 / IMU）与 `robot_localization` EKF 融合
- Nav2 导航与 SLAM 建图
- Gazebo 仿真（需另装 `ros-humble-gazebo-ros-pkgs`）
