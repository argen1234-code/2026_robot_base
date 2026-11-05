@echo off
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0Open-FrameRates.ps1"
if errorlevel 1 pause
