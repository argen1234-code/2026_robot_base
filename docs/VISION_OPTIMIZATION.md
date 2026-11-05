# 树莓派视觉优化保留版（2026-11-05）

## 保留配置

原一键启动命令保持可用。`PERSON_BACKEND=auto` 在 NCNN 模型与运行库均可用时选择 NCNN，否则回退 OpenCV ONNX。当前树莓派选择：

- 相同 YOLOv8n 权重，NCNN CPU 后端，320×320 输入，FP16 存储与运算，1 个推理线程。
- 人体识别上限 6 次/秒；最新彩色帧由独立工作线程处理，ROS 回调仅保存消息。
- 人体标注预览上限 3 帧/秒。手部 JPEG 预览改为 320 像素宽、质量 60、3 帧/秒；手部检测仍使用原始输入和原检测频率。
- 深度历史匹配最接近彩色时间戳的帧；偏差超过 150 ms 时不输出测距位姿。深度采样 ROI 现在也应用配置的偏移量。
- 人体框裁剪同时裁剪两端，修正部分人体在画面边缘时框宽和采样中心偏移的问题。
- 新目标仍需要置信度至少 0.35。最近 0.6 秒内与已确认人体框重叠的当前弱检测可使用 0.2 阈值。没有当前检测时不延用旧框；弱检测不延长确认时间。JSON 显式标记 `confidence_supported`。

## 已验证收益与边界

同一组 197 帧双人画面回放，原 OpenCV 版中位耗时 151.88 ms，NCNN 1 线程为 53.51 ms，约减少 65%；两者每帧均检出两人。NCNN 的 P95 为 70.35 ms，原版为 228.4 ms。输入完全相同，推理后端逐个测试，其他机器人任务保留。

24 帧近距离坐姿样本，原版检出 23 帧，NCNN 320 版检出 24 帧。另一组无人画面，NCNN 32 帧没有检出人体。双人样本没有触发弱检测支持，因此这些结果不能证明该连续性策略在真实遮挡中的收益，也不能推算通用准确率或 mAP。

完整任务运行时，切换前早期样本约 2.86 次/秒；优化后有 4.09、4.72 次/秒的多话题诊断采样，最后电脑端 5 秒窗口为 5.84 次/秒、彩色 9.96 Hz、深度 27.95 Hz。不同场景、窗口和诊断开销不同，这些在线样本不是严格受控的速度倍率测试。识别配置上限 6 Hz，不保证每个窗口均达到。

5 项纯推理/连续性回归检查通过；ROS 检测链路的最新帧、最接近深度时间、过期深度不发布旧位姿、行填充、源时间戳和偏移 ROI 检查通过。没有执行实车移动或真人跟随验收。

深度测距仍存在独立异常：多数有效像素范围不合理，原始 SDK 直读同样出现，Y11/Y12 两种格式均做过只读采样。没有更改相机标定、深度格式或伪造距离；本次保留原 Y11 配置。识别速度提升不代表深度测距问题已解决。

416×320 候选在近距离坐姿样本退步，未作为当前默认模型保留。ONNX Runtime 对比收益小于 NCNN，未用于当前运行。

## 运行库和模型重建

树莓派当前已安装在 `~/vision_runtime`，模型在 `~/models/vision_v2/yolov8n_320_ncnn_model/`。模型文件留在本机，不上传仓库。

```bash
# 使用 ai_host 同一 Python 次版本安装独立运行库，不替换其 OpenCV/NumPy。
~/micromamba/envs/ai_host/bin/python -m pip install --no-deps \
  --target ~/vision_runtime -r tools/requirements-vision-runtime.txt

# 可选：在独立导出环境安装导出依赖（含 PyTorch），然后使用已有的同一权重导出。
~/yolo_export/bin/python -m pip install -r tools/requirements-vision-export.txt
~/yolo_export/bin/python tools/export_person_ncnn.py \
  --weights ~/models/yolov8n_320.pt --runtime-dir ~/vision_runtime
```

导出工具记录输入权重与产物 SHA-256、输入形状和工具版本。若原始 `.pt` 权重不在新机器上，需要先提供相同权重；本工具不会悄悄下载另一个模型。

## 回退与数据

只想切回 ONNX 可使用：

```bash
PERSON_BACKEND=opencv PERSON_MODEL=$HOME/models/yolov8n_320.onnx \
PERSON_INPUT_SIZE=320 PERSON_CV_THREADS=2 PERSON_MAX_FPS=3 \
bash scripts/start_host_stack.sh --map /home/pi/maps/robot_map_20261105_054354 --motor --viz foxglove --person
```

所有实验数据在 `/home/pi/rosbags/codex_vision_20261105/`；修改前备份在 `/home/pi/backup/codex_vision_ncnn_20261105/`。用户要求先保留这一版，后续优化暂停。
