# 电脑端本地帧率监视器

从 Foxglove Bridge 订阅数值统计，显示树莓派本地彩色、深度、人体识别与电脑预览输出频率，不订阅图像。

## Windows 首次安装

需要 Python 3.10+；双击启动器还需要 PowerShell 7（`pwsh`）。在本目录打开终端：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe frame_rates_server.py
```

打开 <http://127.0.0.1:8877/>。之后可以双击 `打开树莓派帧率.cmd`，它会复用已启动的服务或用本目录虚拟环境启动服务。

Linux/macOS 也可以安装 `requirements.txt` 后直接运行 Python 服务。

## 更换树莓派地址或本机端口

```powershell
.\.venv\Scripts\python.exe frame_rates_server.py --bridge-url ws://192.168.31.127:8765 --port 8877
```

启动器默认使用本机 8877 端口和上述树莓派地址；更换参数时请直接运行这条命令。

树莓派需要运行包含 `foxglove_preview.py` 的新版本，并启用 `--viz foxglove --person`。连接中断后服务自动重连。大数字是本地 ROS 数据流实测频率，小数字是树莓派预览副本输出频率。

具体话题、统计口径和验证范围见 [帧率监视说明](../../docs/FRAME_RATE_MONITOR.md)。
