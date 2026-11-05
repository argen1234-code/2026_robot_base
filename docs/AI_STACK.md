# AI 能力栈

## 组成

| 包 | 作用 |
|---|---|
| `interfaces` | 自定义服务/动作（文本、语音、视觉、大模型） |
| `largemodel` | ASR、TTS、云端 LLM 接口、召唤/寻人流程、`nav_task` |
| `perception` | A4 双环目标检测、MediaPipe 手势、双路画面查看 |
| `task_manager` | 目标搜索状态机、速度仲裁 |
| `text_chat` | 文本对话入口 |

## 平台选择

- **文本 LLM**：云端 DeepSeek（`yahboom.yaml: llm_platform`）
- **视觉理解**：云端通义 qwen-vl（`large_model_interface.yaml: vision_platform: tongyi`）
- **ASR**：本地 SenseVoiceSmall（离线、低延迟）
- **TTS**：本地 Piper（中/英）
- **本地大模型**：已停用。实测 `qwen2.5vl:3b` 在 Pi 5 上需约 5.2GB 内存、加载数分钟，不满足实时需求

## 内存预算（8GB 主机）

| 模块 | 估算 |
|---|---|
| 系统 + Docker | 0.8–1.0GB |
| ASR（torch + SenseVoice） | 1.2–1.5GB |
| TTS + MediaPipe + 其他节点 | 0.7–1.2GB |
| 云端视觉 | ≈0 |
| 导航（主机 Jazzy，无 RViz） | 1.2–1.8GB |

运行 AI 时建议关闭 RViz，并给容器设置内存上限（cgroup 恢复后）：
AI 3.5GB / 导航 4GB，各 2 核。

## 混合语音方案（2026-11-05）

本地 ASR/TTS 实测不适合实时：SenseVoice 加载峰值 **3.0GB**、56–118s；
2.21s 语音 ASR 耗时 **8.06s（RTF 3.65）**；Piper 首次合成 **14.5s**。
因此切换为：

- **KWS**：本地 CH340 唤醒模块 `/dev/mic-kws`（udev 别名，雷达仍是 ttyUSB0）；
- **ASR**：云端通义 `paraformer-realtime-v2`（`use_oline_asr: True`）；
- **TTS**：云端 CosyVoice v2（`useolinetts: True`，`tts_supplier: aliyun`）；
- **LLM**：云端 DeepSeek；
- 本地 SenseVoice 不再加载：`funasr` 改为 `init_local_asr_model()` 内延迟导入。

实测 AI 容器 RSS：`model_service` 158MB + `asr` 148MB + launch 33MB
≈ **300MB**，导航可以并行。音频设备是 C-Media USB Audio
（`0d8c:0012`，ALSA card 2，capture 可用），容器内 pyaudio index 0；
容器需要 `/dev/snd`、`/dev/mic-kws`，并挂载主机 PipeWire/PulseAudio
套接字（`SDL_AUDIODRIVER=pulseaudio`），否则 pygame 报 Device busy。

相机仍未连接；`summon_user` 的视觉寻人还需要相机和
`base_link→camera_link` 外参。

## 语音相对导航目标（2026-11-05）

新增 LLM 工具 `move_relative(direction, distance_m, yaw_deg)`：

- 支持“往前走一米 / 向后退半米 / 向左移动 1 米 / 右转 45 度再走 0.5 米”；
- 工具发布 `/nav_goal`（PoseStamped，`frame_id=base_link`，x 前、y 左），
  **AI 容器不需要知道 map/TF**；
- host 侧 `mqtt_nav_egress.py`（domain 2）→ MQTT `robot/ai/nav_goal`
  → `mqtt_nav_ingress.py`（domain 1）→ `/goal_pose`；
- 现有 `goal_sanitizer` 用 TF 把 `base_link` 目标转成 map，再交给
  `bt_navigator`/Nav2；已实测“前方 1 m”被接受（cost=20）。

示例：

```text
你好小鑫，往你的前方走一米
```

测试（AI 容器必须是 domain 2）：

```bash
ROS_DOMAIN_ID=2 ros2 topic pub --once /asr std_msgs/msg/String \
  "{data: '往你的前方走一米'}"
```

`run-ai-service.sh` / `run-ai.sh` 现在用 `AI_ROS_DOMAIN_ID` 固定 AI 域，
默认 2，**不再继承调用 shell 的 `ROS_DOMAIN_ID=1`**；需要同域调试时显式：
`AI_ROS_DOMAIN_ID=1 docker/run-ai-service.sh`。

## 接口

- AI → 导航：MQTT 指令（后续实现），避免 Humble↔Jazzy DDS 跨版本问题
- 召唤流程现有实现：`model_service` → `tools_manager.locate_user()` → 发布 `nav_goal`
- 目标点几何由本地 `person_locator` 计算，云端只负责语义/检测框
