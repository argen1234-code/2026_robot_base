# 2026 Robot Base（单仓库）

机器人底座统一仓库：

| 子项目 | 目录 | 说明 |
|---|---|---|
| 导航（ROS 2 Jazzy） | `robot_ros2_ws/` | 底盘运动学/激光里程计/雷达/建图/Nav2 导航 |
| AI 能力（ROS 2 Humble） | `ai_ws/` | 语音识别/合成、云端大模型、视觉与任务状态机 |
| 相机依赖（第三方） | `third_party/orbbec_ws/` | Orbbec Astra Pro 驱动（锁定上游 commit） |
| STM32 固件 | `stm32-firmware/` | STM32H743 RobStride05 双电机固件（FreeRTOS + CAN + OLED + WS2812 + JDY31 蓝牙） |
| PC 上位机 | `stm32-firmware/上位机/` | RobStride Console（Python 服务 + 浏览器前端 + 单文件 EXE） |
| 微信小程序 | `wechat-mini-program/` | 远程控制小程序（MQTT 桥接） |
| 容器与脚本 | `docker/`、`scripts/` | 环境固化、模型下载、冒烟测试、密钥扫描 |
| 文档 | `docs/` | 迁移记录、AI 栈说明、运行手册 |

> 固件工程来自 [MacSwimmm/2026-AIC](https://github.com/MacSwimmm/2026-AIC)，保留其内部结构、
> 文档与上位机。原 H743 全向底盘固件已移除，可在历史提交 `ed4d5d7` 中恢复。

## 树莓派当前运行方式

`raspberrypi-nav-dev` 分支支持树莓派主机原生启动：导航在 `ros_jazzy` 环境、AI 在 `ai_host` 环境运行，同一 ROS Domain 1，直接通过 ROS 话题交互。

```bash
cd ~/2026_robot_base
bash scripts/start_host_stack.sh --map /home/pi/maps/robot_map_20261105_054354 --motor --viz foxglove --person
```

地图路径按本机实际地图替换；`--motor` 启用电机。环境与设备准备见 [运行手册](docs/RUNBOOK.md)。

- [相机性能与人体识别调整](docs/CAMERA_PERFORMANCE.md)
- [低负载 Foxglove 预览](docs/FOXGLOVE_PREVIEW.md)：原始图像供本地算法使用，电脑预览单独缩图、限帧。
- [本地帧率监视](docs/FRAME_RATE_MONITOR.md)：区分本地彩色/深度/识别频率和预览频率。
- [电脑监视工具](tools/frame_rate_monitor/README.md)：Windows 网页监视器和启动器。

以下保留原有容器部署路线。

## 容器部署的环境分层

- **主机**：ROS 2 Jazzy（RoboStack 或原生），运行导航栈，直连雷达/STM32
- **容器**：ROS 2 Humble，运行 AI 能力（ASR/TTS/云端 LLM/视觉）
- 两者通过 MQTT 交互，避免跨发行版 ROS DDS 通信问题

## 快速开始

```bash
# 1. 密钥（不入库）
cp .env.example .env && chmod 600 .env
# 编辑 .env，填入轮换后的 API Key

# 2. 模型（约 1.1GB，不入库）
scripts/fetch-models.sh

# 3. 构建 AI 工作区（容器内）
scripts/build-ai.sh

# 4. 冒烟测试 / 运行
scripts/smoke-test.sh
docker/run-ai.sh

# 5. 主机 Jazzy 导航环境（RoboStack）与编译
scripts/setup-jazzy-env.sh      # 首次执行，约 1GB 下载
scripts/build-nav.sh            # 编译 YDLidar SDK + robot_ros2_ws
```

## 目录约定

- `ai_ws/` 在容器内挂载为 `/root/yahboom_ws`（部分 install 文件写死该路径）
- 模型放在 `ai_ws/src/largemodel/MODELS/`，被 `.gitignore` 忽略
- `build/`、`install/`、`log/`、`.env`、`project.private.config.json` 均不入库
- 提交前运行 `scripts/check-secrets.sh`

各目录内另有详细 README。
