# 树莓派本地帧率监视

电脑端工具在 [`tools/frame_rate_monitor/`](../tools/frame_rate_monitor/README.md)。按其 README 安装依赖并启动，再打开 <http://127.0.0.1:8877/>。Windows 可双击工具目录的 `打开树莓派帧率.cmd`。启动器会优先使用该目录的 `.venv`。

页面自动连接 `ws://192.168.31.127:8765`。树莓派继续使用原启动命令，Foxglove 断开或机器人任务重启后监视服务会自动重连。没有把监视服务加入 Windows 开机启动。

## 怎么读

- 大数字：树莓派本地原始 ROS 图像消息的实测接收频率，以及人体识别每次完成结果的频率。
- 小数字：树莓派生成电脑预览副本的频率。无人订阅预览时为 0，属于正常行为。
- 人体识别每完成一次就计数，无人画面也计数；它不是画面中人数，也不是模型理论速度。
- 最近 5 秒滑动统计，每秒发布。输入超过 3 秒没有新消息时显示 0；连接中断时显示横线。
- 这是本地 ROS 数据流频率，不是相机硬件内部曝光次数；订阅者遇到调度延迟或 DDS 丢帧时也会反映在读数中。预览输出频率也不等于浏览器实际渲染频率。
- 推理耗时、输入年龄和结果年龄都在树莓派侧测量，不用电脑时钟计算跨机延迟。

## Foxglove 数值面板

新增 `Gauge`（仪表）或 `Plot`（绘图）面板，使用以下路径：

| 内容 | 消息路径 |
|---|---|
| 本地彩色帧率 | `/performance/fps/local_color.data` |
| 本地深度帧率 | `/performance/fps/local_depth.data` |
| 人体识别完成频率 | `/performance/fps/person_detection.data` |
| 彩色预览输出 | `/performance/fps/preview_color.data` |
| 深度预览输出 | `/performance/fps/preview_depth.data` |
| 人体标注图预览输出 | `/performance/fps/preview_person.data` |
| 手部预览输出 | `/performance/fps/preview_hand.data` |
| 单次推理耗时 | `/performance/person_infer_ms.data` |
| 识别输入年龄 | `/performance/person_input_age_ms.data` |
| 识别结果年龄 | `/performance/person_result_age_ms.data` |

单位分别是帧/秒（识别为次/秒）和毫秒。全部为 `std_msgs/msg/Float32`。读取 `data` 数值；统计消息自身固定约 1 Hz，不能把这个 1 Hz 误认为相机帧率。旧 Foxglove 会话如果还没发现新话题，可断开后重连。

## 开销和维护

复用 `foxglove_preview.py`，没有增加 Pi 常驻进程。只有数字被观看时才订阅原始彩色、深度和人体结果做统计；图像回调只保留最新消息并计数，不额外解码。统计本身有少量订阅和计数开销。Windows 网页的 WebSocket 客户端只订阅十个小数值话题，不订阅图像。

本机 Python 服务监听 `127.0.0.1:8877`，网页每秒读取一次。服务代码：`tools/frame_rate_monitor/frame_rates_server.py`；页面：`tools/frame_rate_monitor/web/index.html`。不需要 RustDesk 或 SSH 持续开着。

Pi 源码：`/home/pi/2026_robot_base/robot_ros2_ws/src/robot_base_bringup/scripts/foxglove_preview.py`。

修改前备份：`/home/pi/backup/codex_frame_rates_20261105_201306/foxglove_preview.py`。

定向构建 `robot_base_bringup` 后恢复原配置启动，日志目录：`/home/pi/rosbags/codex_foxglove_transport_20261105_193453/fps_metrics/`。

## 验证记录（2026-11-05）

- 窗口频率计算、断流归零、图像回调不解码、预览尺寸/时间戳/深度毫米值保留，以及只看数字不激活人体/手部标注编码的检查通过。
- `robot_base_bringup` 定向构建通过，安装文件与源文件一致；完整任务恢复 READY。
- 电脑 API 连续返回真实动态数据，最新验证样本为彩色 9.99 Hz、深度 29.33 Hz、人体识别 3.03 Hz；预览约 2.80–2.86 Hz。这是当时的 5 秒窗口，不是保证值。
- 独立本地订阅对照测试收到 130 条彩色、352 条深度和 33 条人体识别结果，三组数值与监视统计在约 5–10% 内接近；采样窗口和订阅 QoS 不同，不能把它当作硬件曝光计数或无丢帧证明。
- 重启后从电脑端仍能收到 `/map`、全局/局部代价地图、`/tf` 和 `/tf_static`。
- HTTP 页面返回 200，启动脚本执行通过并复用了既有监视服务。浏览器已发出打开请求；此处未以 HTTP 检查代替屏幕视觉验收。
