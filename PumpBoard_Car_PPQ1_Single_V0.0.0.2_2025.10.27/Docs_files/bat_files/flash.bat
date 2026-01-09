@echo off
echo ========================================
echo STM32F407ZE Flash Programming Script
echo ========================================
echo.

set BIN_FILE=out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin
set DEVICE=STM32F407ZE
set FLASH_ADDR=0x08000000

if not exist "%BIN_FILE%" (
    echo ERROR: Binary file not found: %BIN_FILE%
    echo Please build the project first using: cbuild PumpCtrl.csolution.yml
    pause
    exit /b 1
)

echo Binary file: %BIN_FILE%
echo Target device: %DEVICE%
echo Flash address: %FLASH_ADDR%
echo.

echo Choose flashing method:
echo 1. ST-Link (STM32_Programmer_CLI)
echo 2. J-Link (JLink.exe)
echo 3. Exit
echo.
choice /C 123 /N /M "Enter your choice (1-3): "

if errorlevel 3 exit /b 0
if errorlevel 2 goto jlink
if errorlevel 1 goto stlink

:stlink
echo.
echo Flashing with ST-Link...
STM32_Programmer_CLI -c port=SWD -w "%BIN_FILE%" %FLASH_ADDR% -v -rst
if %errorlevel% equ 0 (
    echo.
    echo SUCCESS: Firmware flashed successfully!
) else (
    echo.
    echo ERROR: Flashing failed. Make sure ST-Link is connected and drivers are installed.
)
pause
exit /b %errorlevel%

:jlink
echo.
echo Flashing with J-Link...
echo Creating J-Link script...
(
echo device %DEVICE%
echo si SWD
echo speed 4000
echo connect
echo loadbin "%BIN_FILE%" %FLASH_ADDR%
echo r
echo g
echo exit
) > jlink_flash.jlink

JLink.exe -CommanderScript jlink_flash.jlink
if %errorlevel% equ 0 (
    echo.
    echo SUCCESS: Firmware flashed successfully!
) else (
    echo.
    echo ERROR: Flashing failed. Make sure J-Link is connected and drivers are installed.
)
del jlink_flash.jlink
pause
exit /b %errorlevel%
