# Jetson 微信桥接部署

微信小程序 → Jetson 链路的 **Jetson 侧**部署步骤。前置条件：EMQX 云端已建好两个 MQTT 用户并配置白名单 ACL，见 [../emqx/README.md](../emqx/README.md)。

## 前置检查

- EMQX 授权已切换到**白名单模式**（默认拒绝，只放行已授权规则）。
- 已创建两个用户：`robot_001_jetson`（订阅 `get`，发布 `robot/map/path/mission`）和 `robot_001_wechat`（发布 `get`，订阅其余四个）。
- Jetson 能通过系统 CA 信任链访问 EMQX 的 TLS `8883` 端口。

## 1. 拉取最新代码

```bash
cd ~/2026_robot_base
git pull origin main
```

注意：ROS 工作空间目录已更名为 `robot_ros2_ws/`（此前叫 `jetson/`）。

## 2. 安装依赖

```bash
sudo apt install python3-paho-mqtt
# 可选：用于手动验证链路的测试工具
sudo apt install mosquitto-clients
```

## 3. 编译工作空间

```bash
cd ~/2026_robot_base/robot_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. 设置 MQTT 环境变量

桥接从环境变量读取凭据，用户名/密码不写入仓库。三个值缺一（broker / username / password）即报错：

```bash
export WECHAT_MQTT_CLIENT_ID='robot_001_jetson'   # 必须保留，末尾的 _jetson 会被去掉得到 robot_id=robot_001
export WECHAT_MQTT_USERNAME='robot_001_jetson'
export WECHAT_MQTT_PASSWORD='<jetson 用户的 MQTT 密码>'
```

可选覆盖（默认值已可用，一般无需设置）：

| 变量 | 默认值 |
|---|---|
| `WECHAT_MQTT_BROKER` | `i6130f30.ala.cn-hangzhou.emqxsl.cn` |
| `WECHAT_MQTT_PORT` | `8883` |
| `WECHAT_MQTT_CA_CERTS` | 空（用系统 CA） |

## 5. 纯链路测试（不起雷达 / Nav2，不碰电机）

只启动桥接节点，验证「Jetson → EMQX → 订阅方」这一半链路：

```bash
ros2 run robot_base_bringup wechat_bridge.py
```

**成功标志**：日志出现

```text
MQTT connected: /k1ck5t83zdZ/test/user/get (qos=1)
```

此阶段桥接每 1 秒向 `robot` 主题发布一次状态（含 `online: true`），但因无 `map → base_footprint` TF，暂不含坐标 x/y/yaw。

## 6. 验证桥接在发布状态

另开一个 SSH 终端，用 **wechat 身份**订阅状态主题，应每 1 秒收到一条：

```bash
mosquitto_sub --cafile /etc/ssl/certs/ca-certificates.crt \
  -h i6130f30.ala.cn-hangzhou.emqxsl.cn -p 8883 \
  -u robot_001_wechat -P '<wechat 用户的 MQTT 密码>' \
  -t '/k1ck5t83zdZ/test/user/robot' -v
```

持续收到 `{"robot_id":"robot_001","online":true,...}` 即代表 Jetson → EMQX 方向全通。

## 7. 完整启动（带导航，电机按需开启）

有已保存地图（首次联调**关闭电机**）：

```bash
ros2 launch robot_base_bringup navigation.launch.py \
  map:=$HOME/maps/robot_map.yaml motor_enable:=false wechat:=true
```

无地图、需现场建图并联合 Nav2（同样默认不驱动电机）：

```bash
ros2 launch robot_base_bringup mapping_navigation.launch.py \
  motor_enable:=false wechat:=true
```

确认只读链路、定位、路径都正常后，再架空车轮、显式开电机（`motor_enable:=true`），并保持物理急停可用。

## 故障排查

| 现象 | 排查方向 |
|---|---|
| 启动即抛 `WECHAT_MQTT_BROKER, USERNAME and PASSWORD are required` | 漏设了 `WECHAT_MQTT_USERNAME` 或 `WECHAT_MQTT_PASSWORD` |
| 日志 `MQTT connection rejected: rc`（非 0） | 用户名/密码错误，或该用户未在 EMQX 认证里启用 |
| 连上了但 `mosquitto_sub` 收不到 / 报 denied | EMQX 授权未切白名单，或 ACL 规则动作/主题不匹配 |
| TLS 证书报错 | 系统 CA 未更新，或需用 `WECHAT_MQTT_CA_CERTS` 指定证书 |
| 小程序显示在线但无坐标 | 无 `map → base_footprint` TF，先确认 SLAM/AMCL 已发布定位 |
| 小程序里 `robot_id` 对不上 | `WECHAT_MQTT_CLIENT_ID` 结尾必须是 `_jetson` |
