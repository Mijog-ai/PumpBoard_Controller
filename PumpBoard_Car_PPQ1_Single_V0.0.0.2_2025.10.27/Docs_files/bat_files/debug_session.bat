@echo off
echo ========================================
echo STM32F407ZE Debug Session Launcher
echo ========================================
echo.

echo Choose debug method:
echo 1. Keil uVision (Full IDE Debug)
echo 2. OpenOCD + GDB (Command Line)
echo 3. ST-Link Utility (Flash + Monitor)
echo 4. Serial Monitor (UART Debug)
echo 5. Exit
echo.
choice /C 12345 /N /M "Enter your choice (1-5): "

if errorlevel 5 exit /b 0
if errorlevel 4 goto serial
if errorlevel 3 goto stlink_util
if errorlevel 2 goto openocd
if errorlevel 1 goto keil

:keil
echo.
echo Launching Keil uVision...
echo.
echo Instructions:
echo 1. Click Debug -^> Start/Stop Debug Session (Ctrl+F5)
echo 2. Set breakpoints by clicking line numbers
echo 3. Add variables to Watch window
echo 4. Use Peripherals menu to view STM32 registers
echo.
start "" "PumpCtrl.uvprojx"
pause
exit /b 0

:openocd
echo.
echo Starting OpenOCD Debug Session...
echo.
echo Creating OpenOCD config...
(
echo source [find interface/stlink.cfg]
echo source [find target/stm32f4x.cfg]
echo init
echo reset halt
) > openocd.cfg

echo.
echo Starting OpenOCD server...
start "OpenOCD" cmd /k "openocd -f openocd.cfg"
timeout /t 2 /nobreak >nul

echo.
echo Starting GDB...
echo.
echo GDB Commands:
echo   target remote localhost:3333
echo   monitor reset halt
echo   load out/PumpCtrl/PPQ_Ctrl/PumpCtrl.axf
echo   break main
echo   continue
echo.
arm-none-eabi-gdb out/PumpCtrl/PPQ_Ctrl/PumpCtrl.axf
exit /b 0

:stlink_util
echo.
echo Launching ST-Link Utility...
echo.
echo Instructions:
echo 1. Connect to target
echo 2. Use Memory view to monitor variables
echo 3. Use MCU Core view to see registers
echo.
start "" "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_Utility.exe"
pause
exit /b 0

:serial
echo.
echo Serial Monitor Setup
echo ========================================
echo.
echo To enable serial debug output:
echo 1. Connect UART pins (TX=PA9, RX=PA10 typically)
echo 2. Use USB-to-Serial adapter
echo 3. Configure: 115200 baud, 8N1
echo.
echo Available serial monitor tools:
echo - PuTTY
echo - TeraTerm
echo - Arduino Serial Monitor
echo - HTerm
echo.
echo Opening Windows Device Manager to find COM port...
start devmgmt.msc
echo.
echo After finding COM port, launch your serial monitor.
echo.
pause
exit /b 0
