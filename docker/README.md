# 容器环境

## 镜像

| 镜像 | 说明 |
|---|---|
| `yahboom/ros-humble-ai:base-v1.0` | 原始基础镜像快照（导入镜像，无法从零复现） |
| `yahboom/ros-humble-ai:snapshot-20261104` | AI 容器 `docker commit` 快照（含 ASR/TTS/LLM 依赖与临时安装） |
| `yahboom/ros-humble-ai:2026.11` | 由 `Dockerfile` 从 snapshot 派生（entrypoint/label） |

> 导航容器镜像 `yahboom/ros-nav:snapshot-20261104` 已于 2026-11-05 删除；
> 导航统一在主机 `ros_jazzy` 环境运行，不再使用 Humble 回退容器。

完整依赖清单见 `base-image-manifest.txt`。

## 构建

```bash
docker build -f docker/Dockerfile -t yahboom/ros-humble-ai:2026.11 docker/
```

> 构建上下文是 `docker/`，避免把 1GB 模型和 96MB 固件发送给 daemon。

## 运行

```bash
# AI 交互
docker/run-ai.sh
# AI + 指定 launch
docker/run-ai.sh ros_humble_ai_dev ros2 launch largemodel largemodel_control.launch.py
# 后台 AI 服务
docker/run-ai-service.sh
```

## 挂载与路径约定

- 仓库 `ai_ws/` 挂载为容器内 `/root/yahboom_ws`（部分 install 文件写死该路径）
- `third_party/orbbec_ws/` 挂载为 `/root/orbbec_ws`
- 模型在 `ai_ws/src/largemodel/MODELS/`（不入库，用 `scripts/fetch-models.sh` 下载）
- 密钥通过 `.env` 注入（`--env-file`），不写进镜像

## 内存限额

当前内核对 memory cgroup 禁用了，脚本会检测并跳过 `--memory`。
启用后（重启并恢复 cgroup）会自动生效：AI 3.5G、导航 4G、各 2 核。
