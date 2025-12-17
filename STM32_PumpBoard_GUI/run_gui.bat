@echo off
REM Launcher script for STM32 PumpBoard GUI (Windows)

echo Starting STM32F407 PumpBoard Controller GUI...
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.7 or higher from https://www.python.org/
    pause
    exit /b 1
)

REM Check if required packages are installed
python -c "import PyQt5; import pyqtgraph; import serial" >nul 2>&1
if errorlevel 1 (
    echo Installing required Python packages...
    pip install -r requirements.txt
    if errorlevel 1 (
        echo Error: Failed to install dependencies
        echo Please manually run: pip install -r requirements.txt
        pause
        exit /b 1
    )
)

REM Launch the GUI
python gui_main.py

pause
