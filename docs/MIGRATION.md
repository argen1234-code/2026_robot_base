# 迁移记录：单仓库收敛

## 变化

| 原位置 | 新位置 |
|---|---|
| `/home/pi/yahboom_ws/src/` | `ai_ws/src/` |
| `/home/pi/yahboom_ws/astra_ws/src/OrbbecSDK_ROS2_main/` | `third_party/orbbec_ws/src/OrbbecSDK_ROS2/`（去掉上游 `.git`，锁定 `0ea4ab6`） |
| `/home/pi/yahboom_ws/astra_ws/install/` | `third_party/orbbec_ws/install/`（本地预编译，不入库） |
| 散落的 `ros2_docker.sh` 等 | `docker/`、`scripts/` |

旧目录保留为 `/home/pi/yahboom_ws.old-20261104`，`/home/pi/yahboom_ws` 现在是指向
`2026_robot_base/ai_ws` 的符号链接，保证旧脚本/旧容器路径继续可用。

## 回滚

```bash
docker stop ros_humble_ai_dev
rm /home/pi/yahboom_ws
mv /home/pi/yahboom_ws.old-20261104 /home/pi/yahboom_ws
docker start ros_humble_ai_dev
```

## 模型

模型不进入 git。新机器上执行：

```bash
scripts/fetch-models.sh
```

包含 SenseVoiceSmall（ASR）与两个 Piper 语音（中/英）。

## 密钥

所有 API Key 已从 YAML 移到 `.env`，仓库只保留 `.env.example`。
迁移前的旧 Key 视为已泄露，需在服务商控制台轮换。
