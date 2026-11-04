@echo off
chcp 65001 >nul
cd /d "%~dp0"
if exist "..\RobStrideConsole.exe" (
    start "" "..\RobStrideConsole.exe"
    exit /b
)
python server.py
if errorlevel 1 (
    echo 启动失败。请先运行：python -m pip install -r requirements.txt
    pause
)
