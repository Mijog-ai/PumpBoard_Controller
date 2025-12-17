#!/bin/bash
# Launcher script for STM32 PumpBoard GUI

echo "Starting STM32F407 PumpBoard Controller GUI..."
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    echo "Please install Python 3.7 or higher"
    exit 1
fi

# Check if required packages are installed
python3 -c "import PyQt5; import pyqtgraph; import serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required Python packages..."
    pip3 install -r requirements.txt
    if [ $? -ne 0 ]; then
        echo "Error: Failed to install dependencies"
        echo "Please manually run: pip3 install -r requirements.txt"
        exit 1
    fi
fi

# Launch the GUI
python3 gui_main.py
