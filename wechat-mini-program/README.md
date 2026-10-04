# 微信小程序：跨设备导入与 MQTT 配置

小程序源码和运行所需的 `utils/mqtt.wx.js` 已包含在仓库中。微信开发者工具导入后不需要执行 `npm install` 或「构建 npm」。当前功能为遥控和室内 Nav2 导航；GPS 页面源码暂未接入 Jetson GPS 后端。

## 从 GitHub 导入

1. 在另一台电脑克隆 `https://github.com/argen1234-code/2026_robot_base.git`，切换到 `main`。
2. 打开微信开发者工具，选择「导入项目」，将项目目录选为克隆仓库内的 `wechat-mini-program/`（不要选择仓库根目录）。选择小程序项目，并使用自己有权限的 AppID；仓库的 `project.config.json` 带有已有 AppID `wx888840c0a38a8ed7`，如无其权限请在开发者工具中更换为自己的 AppID。微信后台的 socket 合法域名配置必须属于实际使用的 AppID。
3. 导入后应能直接编译到「状态」页，即使没有 MQTT 凭据也能显示「待配置账号」；这时不能遥控小车。不要把账号或密码写入 `project.config.json`、`api.js`、截图或 Git 提交。
4. 先在 EMQX 创建**专用小程序 MQTT 用户**，例如 `robot_001_wechat`，并按 [EMQX 授权说明](../deploy/emqx/README.md)配置规则。Jetson 应使用另一个用户 `robot_001_jetson`。控制台登录邮箱密码和 API Key 都不是 MQTT 客户端凭据。
5. 在小程序「状态」页输入小程序 MQTT 用户名及密码，点击「保存并连接」。每台设备须各自输入一次；密码保存在本设备微信本地存储，不跟随 Git 同步。再次进入页面只展示用户名，不回填密码；修改用户名或密码时重新输入密码，点击「清除账号」会断开连接并清除本机凭据。共享手机不建议存储能控制小车的账号，泄漏时在 EMQX 撤销或更换密码。

连接地址在 `utils/api.js`：`wxs://i6130f30.ala.cn-hangzhou.emqxsl.cn:8084/mqtt`。客户端会订阅 `/k1ck5t83zdZ/test/user/robot`、`map`、`path`、`mission`，只向 `/k1ck5t83zdZ/test/user/get` 发布命令。状态页的「MQTT 已连接」只证明与 Broker 连通；须同时看到机器人状态更新、地图/路径/任务数据才能确认 Jetson 桥接在线。

## 真机前的外部配置

- 微信公众平台该 AppID 下，在「开发管理 → 开发设置 → 服务器域名」添加 `wss://i6130f30.ala.cn-hangzhou.emqxsl.cn` 为 **socket 合法域名**。开发者工具里的 `urlCheck: false` 仅用于本机调试，不能代表真机合法域名已生效。
- 当前 WSS 服务使用 8084 端口。微信正式环境对端口/域名的校验与开发者工具不同，必须在真机实测；若该端口被平台拒绝，需要由有域名/代理配置权限的一方提供 443 上的 WSS 入口，再修改 `utils/api.js` 的 `BROKER_URL`。这一步不能通过小程序代码绕过。
- Jetson 桥接默认关闭，需要为 Jetson 设置其专用 MQTT 用户名密码，并以 `wechat:=true` 启动对应 ROS launch。首次测试保持 `motor_enable:=false`，不要发送运动命令。

导入编译、真机 WSS/鉴权及只读联调的验收项记录于仓库根目录 [WECHAT_MINIPROGRAM_ACCEPTANCE.md](../WECHAT_MINIPROGRAM_ACCEPTANCE.md)。
