# 2026 Robot Base（单仓库）

机器人底座统一仓库：

| 子项目 | 目录 | 说明 |
|---|---|---|
| 导航（ROS 2 Jazzy） | `robot_ros2_ws/` | 底盘运动学/激光里程计/雷达/建图/Nav2 导航 |
| AI 能力（ROS 2 Humble） | `ai_ws/` | 语音识别/合成、云端大模型、视觉与任务状态机 |
| 相机依赖（第三方） | `third_party/orbbec_ws/` | Orbbec Astra Pro 驱动（锁定上游 commit） |
| STM32 固件 | `stm32-firmware/` | STM32H743 四轮全向小车（FreeRTOS + LVGL + GPS） |
| 微信小程序 | `wechat-mini-program/` | 远程控制小程序（MQTT 桥接） |
| 容器与脚本 | `docker/`、`scripts/` | 环境固化、模型下载、冒烟测试、密钥扫描 |
| 文档 | `docs/` | 迁移记录、AI 栈说明、运行手册 |

## 环境分层

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
```

## 目录约定

- `ai_ws/` 在容器内挂载为 `/root/yahboom_ws`（部分 install 文件写死该路径）
- 模型放在 `ai_ws/src/largemodel/MODELS/`，被 `.gitignore` 忽略
- `build/`、`install/`、`log/`、`.env`、`project.private.config.json` 均不入库
- 提交前运行 `scripts/check-secrets.sh`

各目录内另有详细 README。
