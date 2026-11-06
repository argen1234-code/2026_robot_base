# 蓝牙优先于 ROS USB 控制（2026-11-05）

蓝牙使用 USART1 / PA9、PA10，9600 8N1，命令必须带 @ 前缀和真实 CRLF。

- `@ARM`：即使 ROS USB 已使能或正在准备，也会先归零并撤销 USB 授权，再完成蓝牙零速接管。等待 `OK ARM` 后发送方向。
- `@F` 等方向每 150–200 ms 重复发送；`@S` 停车，蓝牙已使能时保持授权。
- 蓝牙接管后，USB 的速度帧、ARM 和 DISARM 不能改变蓝牙控制；USB 遥测继续。
- `@!` / `@DISARM`：失能并保持人工控制锁。运动 600 ms 无续发、UART 错误或故障也会停车并阻止 ROS 自动恢复。
- `@RELEASE`：明确退出蓝牙并解除人工控制锁；此后 ROS 可通过新的 ARM 及后续新速度帧恢复。RELEASE 本身不会恢复旧速度。
- 全局停止和电机/CAN保护继续生效。SWD底盘控制仍沿用原互斥规则。

注意：RELEASE 不再是 DISARM 的别名。蓝牙 ARM 后若需要返回导航，必须显式 RELEASE。

验证：真实电机、蓝牙与USB应用代码的host联合测试通过，包含运动/准备中接管、USB干扰隔离、STOP、超时锁、RELEASE、故障及全局停止。GCC构建及OpenOCD Flash verify通过；实板USB计时/零目标/IMU遥测正常。手机与JDY31物理蓝牙接管仍待实际验证。

详细构建、测试、烧录和USB实板日志：`/tmp/bt-priority-20261105/`。原文件：`/tmp/bt-priority-backup-20261105/`；原固件产物：`/tmp/bt-priority-20261105/original-firmware/`。临时目录可能在重启后清理。
