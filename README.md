# 2026 Robot Base（三端工程）

机器人底座统一仓库，包含三个子项目：

| 子项目 | 目录 | 说明 |
|---|---|---|
| Jetson（ROS 2） | `robot_ros2_ws/` | 底盘 ROS 2 工作空间（Jazzy），运动学/里程计/激光雷达/单雷达建图与 Nav2 导航 |
| STM32 固件 | `stm32-firmware/` | STM32H743 四轮全向小车（FreeRTOS + LVGL + GPS） |
| 微信小程序 | `wechat-mini-program/` | 机器人远程控制小程序（遥控/室内导航，通过 MQTT 桥接 Jetson） |

各子项目的详细说明见各自目录内的 `README.md`。
