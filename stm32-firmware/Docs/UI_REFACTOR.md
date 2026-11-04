# 2026 AIC 上位机 UI 重构与验证

日期：2026-11-01。

## 来源与范围

- 完整工程：[MacSwimmm/RobStride05](https://github.com/MacSwimmm/RobStride05)，基线提交 `4014edb2e63edcddbc20475b6d9c74c3eb2e1aab`。
- 视觉与交互模板：[MacSwimmm/upper-computer-template](https://github.com/MacSwimmm/upper-computer-template)，参考提交 `4e2d8f96411601768db7d591bb1a651ef17ab1f9`。
- 新仓库：`MacSwimmm/2026-AIC`。GitHub 仓库名使用连字符；项目显示名为“2026 AIC”。
- 使用指定 GitHub 仓库的已提交版本。原本本地其他目录中的未提交改动不在此次导入范围。
- 保留完整 Git 历史和根目录布局。未向原仓库或模板仓库推送。

模板只提供外观和交互参考。没有引入任务调度示例后端、仿真数据或仿真按钮；生产界面继续显示真实 SWD 状态，未连接时保持空值。

## 功能对应

| 原有功能 | 新界面入口 |
|---|---|
| 探针刷新、连接/断开、Flash 身份核对 | 工作空间顶部连接条 |
| 双电机切换、独立参数、ID 127/126 配置及核对 | 两张电机卡片、首次接入折叠区 |
| 准备、使能、正反转、运行、应用速度、减速停止、单台失能 | 电机控制模块 |
| 两台全部失能、Esc | 全局双电机区；Esc 在设置抽屉内也保留 |
| 速度、位置、力矩、温度、电流、电压 | 电机控制模块的状态和读数 |
| 30 秒多指标曲线、目标/实际切换、暂停、CSV | 数据观测模块 |
| CAN/RTOS/参数回执/固件身份、清错 | 通信诊断模块 |
| 七种灯效、RGB、亮度、速度、开关、颜色顺序 | 灯带控制模块 |
| 最多 8 项循环、排序、重复、时长、本机保存/读取 | 灯带控制模块 |
| 日志复制、使用指南、退出释放探针 | 日志区、使用指南、右上角退出 |

卡片切换仅改变展示，不发送运动命令；非当前卡片内容设置为 `inert`，防止通过侧面预览误操作。数字框、滑块、下拉框内的方向键保留输入行为。触摸切换限定为标题栏和卡片说明，避免与参数滑块冲突。设置抽屉具有焦点约束、关闭后焦点恢复，并跟随系统减少动态效果设置。

## 保留范围

完整保留 `RobStride05_H743/` 中全部原始文件，包括原 AXF/HEX、厂商库、协议、RTOS、OLED、灯带、CubeMX 集成脚本和历史验证材料。后端 `Controller`、参数验证、队列、回执和数据导出不变；`bridge.py`、`profile.py` 与原有灯带逻辑不变。`server.py` 仅在静态白名单增加 `/template.css`、`/template-ui.js`，保留 Host、Origin、会话令牌和 CSP。

`web/app.js` 仅调整 `switchView()` 的展示委托与曲线在数据观测模块内的可见条件。原按钮 ID、事件处理、命令提交、通道选择、状态门禁和速度单位转换继续复用。新增样式和界面逻辑在 `web/template.css`、`web/template-ui.js`。

打包时 `firmware_profile.json` 从交付的原 AXF 重新提取；ABI、Flash 段与固件 SHA 保持一致，仅记录的本地来源路径会更新。已替换单文件 EXE，避免源码界面和双击程序版本不一致。

## 验证与证据

1. 原有 Python 测试：`python -m unittest discover -s tests -p 'test_*.py' -v`，28 项通过。
2. 原有 C 主机检查：`pwsh -File ./RobStride05_H743/Tools/test_host.ps1`，双电机隔离/并发/失能、OLED 布局与字形、灯带循环/DMA 等通过。
3. Keil 在仓库外独立副本重新执行 `Tools/build.ps1`：ARMCC 5.06u7，`0 Error(s), 0 Warning(s)`。没有用重编译产物覆盖原固件文件，也未烧录。
4. `tests/check_template_ui.py`：生产服务离线状态检查五模块、双电机编辑与单位换算、ID 配置门禁、LED 编辑/保存、设置焦点与持久化、1920/1280/768/390 宽度、触摸、普通动效和减少动效。HTTP 拦截夹具验证控制命令、电机通道、清错、Esc、CSV 和灯效提交。不会打开真实硬件连接。
5. PyInstaller 单文件重打包。只复制 EXE 到仓库外临时目录启动，在无相邻资源文件和 AXF 的情况下运行同一浏览器回归，验证内置静态资源和固件档案。
6. 文件保留与 API 实现比较见 [preservation.json](../上位机/RobStride_Console/review/preservation.json)；浏览器明细见 [ui-regression.json](../上位机/RobStride_Console/review/ui-regression.json)。打包与固件重编译日志位于同一 `review/` 目录。

本轮未做硬件转动、物理灯效、负载或温升验收；原有实板 JSON、照片、CSV 与文档保留作为历史证据。文件与软件回归通过并不增加新的实机验收结论。`review/06-fixture-led.png` 的连接状态来自自动化夹具，不是实机截图。

## 复现

仓库根目录：

```powershell
python -m pip install -r ./上位机/RobStride_Console/requirements.txt
python -m pip install playwright pyinstaller
Push-Location ./上位机/RobStride_Console
python -m unittest discover -s tests -p 'test_*.py' -v
python server.py --port 8876 --no-browser
# 在另一个终端、同一上位机源码目录执行：
python tests/check_template_ui.py --url http://127.0.0.1:8876
# 需要重新打包时：
pwsh -File ./build-singlefile.ps1
Pop-Location
```

UI 自动化使用本机 Microsoft Edge，不需连接调试探针。

模板仓库未声明统一许可证，本仓库保留来源说明；STM32 HAL、CMSIS、FreeRTOS 等原版权和许可说明保持原样，未为全部内容另行添加统一许可证。
