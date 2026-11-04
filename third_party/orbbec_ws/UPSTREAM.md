# Orbbec SDK ROS 2（第三方）

- 上游：https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2.git
- 分支：`main`
- 锁定提交：`0ea4ab68f5b8ddf22998b7c45d251dc58ea2779e`（2026-08-31）
- 许可证：见 `src/OrbbecSDK_ROS2/LICENSE` 与 `NOTICE`

## 目录说明

- `src/OrbbecSDK_ROS2/`：源码快照（已去掉上游 `.git`），入库
- `install/`、`build/`：本地预编译产物，**不入库**（`.gitignore` 已忽略）

## 更新方式

```bash
cd /tmp
git clone https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout <new-commit>
rsync -a --exclude='.git/' ./ \
  /home/pi/2026_robot_base/third_party/orbbec_ws/src/OrbbecSDK_ROS2/
```

重新编译：

```bash
/home/pi/2026_robot_base/scripts/build-orbbec.sh
```

注意：本设备是 legacy Astra Pro，使用上游 `main`/v1.x 分支，不要切换到 v2-main。
