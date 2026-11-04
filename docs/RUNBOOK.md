# 运行手册

## 设备准备

```bash
# 雷达/STM32 的稳定设备名（按本机 USB 拓扑测量后修改 devpath）
sudo install -m 0644 \
  robot_ros2_ws/src/robot_base_bringup/udev/99-robot-base.rules \
  /etc/udev/rules.d/99-robot-base.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/ydlidar /dev/stm32
```

> 仓库模板里的 `devpath` 来自原 Jetson，树莓派上必须用
> `udevadm info -a -n /dev/ttyACM0` 重新取值。

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

## 启动顺序

1. 主机：进入 `ros_jazzy` 环境
2. 导航：`ros2 launch robot_base_bringup robot_base.launch.py`
3. AI 容器：`docker/run-ai.sh`
4. 容器内：`ros2 launch largemodel largemodel_control.launch.py`

## DDS 隔离

| 栈 | ROS_DOMAIN_ID |
|---|---|
| 导航（主机） | 1 |
| AI（容器） | 2 |

脚本已默认设置，避免两个栈话题串台。

## 常见问题

| 现象 | 排查 |
|---|---|
| 容器看不到串口 | 启动参数是否包含 `--device`；`/dev/ydlidar` 软链接是否在宿主创建 |
| `No memory limit support` | 内核禁用了 memory cgroup；earlyoom + `ulimit -v` 兜底 |
| 云端 LLM 无响应 | `.env` 中的 Key 是否已轮换并生效 |
| 视觉不可用 | `vision_platform` 是否为 `tongyi`；云端网络是否可达 |
| 旧路径脚本失效 | `/home/pi/yahboom_ws` 是指向 `ai_ws` 的符号链接，确认链接存在 |

## 内存看门狗

- earlyoom 常驻：可用内存 ≤10% / swap ≤20% 先 SIGTERM，≤5% 时 SIGKILL
- 编译/重活：`ulimit -v 4194304` + `--parallel-workers 2`
- cgroup 恢复后：`docker info` 不再出现 `No memory limit support`，运行脚本会自动加内存上限
