# perception

A4 双同心圆环目标检测（当前阶段：Orbbec UVC RGB + legacy 深度驱动）。

## 目标几何（实测打印件）

```text
外黑环：外径 167 mm，内径 127 mm，环宽 20 mm
白色分隔环：外径 127 mm，内径 108 mm，宽 9.5 mm
内黑环：外径 108 mm，内径 66 mm，环宽 21 mm
中心白圆：直径 66 mm
四个半径：83.5 / 63.5 / 54 / 33 mm
```

相机安装：高度约 50 cm，相对底盘中心前移 12 cm，上仰（角度待实测确认）。
`config/ring_detector.yaml` 中的目标尺寸已按实测值更新。

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/yahboom_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# 合成图 / 指定图片离线检测
ros2 run perception ring_detector_demo
ros2 run perception ring_detector_demo --image /path/to/photo.jpg

# 实时相机节点
ros2 launch perception ring_detector.launch.py

# 用图片验证 ROS 发布链路（不需要相机）
ros2 run perception ring_detector_node --ros-args \
  -p test_image:=/root/yahboom_ws/src/perception/test_images/target_real.jpg
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/target_pose` | `geometry_msgs/PoseStamped` | 目标中心，`camera_optical_frame` |
| `/perception/target_debug` | `std_msgs/String` | JSON：valid/method/center/radii/distance |
| `/perception/debug_image` | `sensor_msgs/Image` | 标注后的图像 |

## Orbbec 驱动（legacy Astra Pro）

这颗设备是 legacy OpenNI 的 **Astra Pro**（PID 0x0403，SN 18043020197，
固件 RD107E-007）。v2-main 驱动会报 `propertyId: 1000`，无法初始化；
必须使用 `main`（v1.x / SDK 1.10.37）分支。

- 源码：`~/yahboom_ws/astra_ws/src/OrbbecSDK_ROS2_main`
- 编译产物：`~/yahboom_ws/astra_ws/install`
- udev 规则：已安装 `/etc/udev/rules.d/99-obsensor-libusb.rules`

### 深度-only 启动

Astra Pro 的彩色流由独立的 UVC HD 相机提供；legacy OpenNI 驱动启动彩色流会报
`Match openni video mode failed`，所以彩色继续走 `/dev/video0`，驱动只开深度：

```bash
source /opt/ros/humble/setup.bash
source /root/yahboom_ws/install/setup.bash
source /root/yahboom_ws/astra_ws/install/setup.bash

ros2 launch orbbec_camera astra.launch.py \
  enable_color:=false enable_depth:=true \
  depth_width:=320 depth_height:=240 depth_fps:=30 depth_format:=Y11
```

需要 IR 时：

```bash
ros2 launch orbbec_camera astra.launch.py \
  enable_color:=false enable_depth:=true enable_ir:=true \
  depth_width:=320 depth_height:=240 depth_fps:=30 depth_format:=Y11 \
  ir_width:=640 ir_height:=480 ir_fps:=30 ir_format:=Y10
```

话题：

- `/camera/depth/image_raw`（约 29.7 Hz）
- `/camera/depth/camera_info`（frame: `camera_depth_optical_frame`）
- `/camera/depth/points`
- `/camera/ir/image_raw`（启用 IR 时，mono16 640×480）

> 限制：这颗设备的 legacy 驱动给出的 `camera_info.K` 为 0/NaN。可先用
> Astra Pro 近似内参（640×480 时 fx≈577、fy≈580、cx=320、cy=240；
> 320×240 时按 0.5 缩放），精确值需要标定。圆环距离仍可由目标已知尺寸的
> PnP 提供，深度可用于大致距离/障碍校验。

容器如果在相机插入之前启动，`/dev/video0` 可能不存在：

```bash
bash /root/yahboom_ws/src/perception/scripts/setup_orbbec_uvc.sh
```

## 下一步

1. 把深度/IR 相机对准目标，确认 IR 图像里双环是否有足够对比度；
2. 用近似内参或标定值，把深度像素转成 3D 点；
3. 在 `ring_detector_node` 中增加可选的 depth 订阅与距离校验；
4. 相机内参标定后替换 `camera_fx/fy/cx/cy`；
5. 继续台架测试 `task_manager` 的 search → approach → arrived。
