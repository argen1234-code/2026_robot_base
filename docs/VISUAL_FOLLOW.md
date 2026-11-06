# 独立纯视觉找人模式

这条模式只使用彩色相机的人框和深度相机的人体距离：

```text
Astra 彩色图像 → 现有 YOLO 人框 → 水平中心偏差 → 角速度
Astra 深度图像 → 人框内有效深度 → 前进 / 停车
                                      ↓
                       /remote_cmd_vel + /robot_mode=2
                                      ↓
                       原有 chassis_bridge → STM32 USB
```

不启动 Nav2、SLAM、雷达、地图定位、导航决策、语音或手部检测。
`robot.sh` 启动和停止仍统一管理这些组件。旧 `--task follow` 保留原来的导航行为。

## 使用

在树莓派终端执行，先进入工程：

```bash
cd ~/2026_robot_base

# 无电机预览：真实相机与检测运行，控制器不发布速度或模式命令
bash scripts/robot.sh start --task visual-follow --distance 0.05 --restart
```

网页：`http://192.168.43.14:8879`，IP 变化时替换为树莓派当前地址。
页面显示人框、左右偏差、人体深度、控制状态和实际输出。
预览时点按钮只预览计算结果；电机输出固定为零。

启用电机的两个模式（启动即执行任务）：

```bash
# 靠近一次：有效深度 <= 指定距离 + 0.02，停车并结束；再次靠近需重新点击开始
bash scripts/robot.sh start --task visual-approach --motor --distance 0.05 --restart

# 持续保持距离：到距离停车，人拉开 0.05 m 后继续；过近时停车，不倒退
bash scripts/robot.sh start --task visual-follow --motor --distance 0.05 --restart

# 停止整栈（包含底盘桥的零速/解除使能）
bash scripts/robot.sh stop
```

页面也可选择「靠近一次」「保持距离跟随」，并修改停车深度。
点「停止」结束当前任务；`/stop_request` 和控制模式切换也会停止任务。
没有选择新任务不会自动恢复。

## 距离与当前相机的实际限制

距离是**深度镜头光学坐标的 Z 值**，不是车头到人的间隙；不使用雷达或地图坐标。

2026-11-07 USB 枚举确认当前设备是 `2bc5:0403 Orbbec Astra Pro`。
厂商 [Astra 系列规格](https://www.orbbec.com/products/structured-light-camera/astra-series/)
列出的深度范围是普通型号 **0.6–8 m**，S 型号 **0.4–2 m**。
**0.05 m 不在当前型号标称测距范围内，软件无法保证在此处获得有效深度。**
本模式按需求保留 0.05 m 默认值，并把检测器有效深度的软件下限设为 0.01 m，
避免过去的 `>0.30 m` 过滤规则直接排除 0.05 m。但这只修正软件过滤，
不能扩展硬件量程。建议当前相机首先使用 `--distance 0.70` 检查实车跟随。
要验收真实 0.05 m 停车，需要确认相机实测支持该距离或使用支持此近距的深度相机。

彩色/深度 ROI 目前沿用原检测器的归一化映射，可通过检测器
`--depth-offset-x/y` 微调，不宣称已完成两路镜头标定。

## 控制行为

- 人框偏右向右转，偏左向左转；偏差超过画面宽度 18% 时停止前进、先居中。
- 最高前进 0.24 m/s，最高转向 0.55 rad/s；距离增益 1.8，居中增益 2.8。
  按当前轮半径 0.041713 m，直行最高目标轮速约 5.75 rad/s。
- 人框有效且尚未到达时，即使人体深度无效、深度流缺失/过期或与 RGB 不同步，
  都继续按人框居中，并以 0.07 m/s 缓慢接近；曾经测到深度也不改变此行为。
  深度恢复新鲜、同步且有效后，重新按距离调速和判断到达；无效或旧深度不能触发到达。
- 「靠近一次」停车死区 `approach_deadband_m=0.02`：首次收到有效深度
  <= 页面停车阈值 + 0.02，立即锁定到达，前进和转向同时归零。
  例如页面设置 0.05，则深度读数 <= 0.07 就结束，不再在 0.05 附近慢慢挪动。
  「保持距离跟随」仍直接使用页面阈值，不加该死区。
  检测回调直接发零速，底盘桥收到运动到零速的切换后直接写 USB 零速帧，
  不再等待两层各 50 ms 的周期定时器；后续深度回跳、失效都不会重新开始移动。
  网页保留触发停车的深度读数，只有再次点击开始才清除到达状态。
- 持续跟随已经由有效深度触发保持停车时，深度丢失仍保持前进零速、继续居中，
  直到有效深度确认人拉开到恢复距离；单次到达仍锁定结束。
- 没有人框、旧彩色图像或无效检测时停车；深度缺失本身不会触发停车。
- 首次选择最靠近画面中心的人；之后用人框重叠维持目标。多人歧义时停车。
  这是人框连续性，不能保证身份重识别；失去目标后可重新点击开始选择目标。
- 保留固件现有 USB 看门狗、蓝牙优先权和故障处理。蓝牙手动 hold 尚未 RELEASE 时，
  固件仍可能拒绝 USB 控制；本模式不会强行抢占。

配置文件：`config/visual_follow.json`。新节点为 `robot_base_decision/visual_follow`。
`image_yaw_sign` 是这台实车的画面转向极性。2026-11-07 实测：正 ROS 角速度
使背景特征向画面左侧移动，负角速度使其向右移动。因此当前配置为 `+1`：
目标偏右时发正角速度，目标偏左时发负角速度，使偏差缩小。
该标定只作用于视觉误差到角速度的映射；底盘协议、导航和蓝牙映射不变。
更换或翻转摄像头后应重新观察静止场景对短时转向的响应，不能仅凭角速度名称判断方向。
状态话题 `/visual_follow/status`，HTTP `GET /status`、`GET /image.jpg`，
命令 `POST /start`、`POST /follow`、`POST /stop`。

## 验证

测试覆盖控制/取深度、转向、真实 ROS 收发、假 USB 停车和启动入口，
包括测到深度后再丢失仍前进、深度流中断仍居中，以及有效到达后保持停车。
ROS 测试使用独立 domain 79；启动测试用假组件，均不驱动真实电机。

```bash
cd ~/2026_robot_base
PYTHONPATH=$PWD/robot_ros2_ws/src/robot_base_decision \
  python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
  -p test_visual_follow.py -v
python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
  -p test_robot_visual_start.py -v
~/.local/bin/micromamba run -n ros_jazzy bash -c \
  'source robot_ros2_ws/install/setup.bash; \
   PYTHONPATH="$PWD/robot_ros2_ws/src/robot_base_decision:$PYTHONPATH" \
   python3 -m unittest discover -s robot_ros2_ws/src/robot_base_decision/test \
   -p test_visual_follow_ros.py -v'
```

代码/ROS 测试通过不代表实车已达到 0.05 m；停车精度、左右转向、镜头近距量程和
真实人框深度关联须用实车确认。
