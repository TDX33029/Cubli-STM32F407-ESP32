@echo off
chcp 65001 >nul
title Cubli 平衡控制上位机
echo ===================================================
echo   Cubli 平衡控制上位机 (PyQt5)
echo ===================================================
echo 正在启动上位机...
cd /d "%~dp0"
python main.py
if errorlevel 1 (
    echo.
    echo [错误] 程序异常退出。请检查是否安装了所需依赖:
    echo        pip install -r requirements.txt
    pause
)
