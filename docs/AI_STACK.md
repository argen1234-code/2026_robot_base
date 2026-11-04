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

## 接口

- AI → 导航：MQTT 指令（后续实现），避免 Humble↔Jazzy DDS 跨版本问题
- 召唤流程现有实现：`model_service` → `tools_manager.locate_user()` → 发布 `nav_goal`
- 目标点几何由本地 `person_locator` 计算，云端只负责语义/检测框
