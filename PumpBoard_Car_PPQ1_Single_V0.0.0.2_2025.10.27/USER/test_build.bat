@echo off
echo ========================================
echo Hardware Test Build Script
echo ========================================
echo.

echo Select test mode:
echo 1. GPIO Test (LED Blink)
echo 2. ADC Test (Read Analog Input)
echo 3. PWM Test (Timer Output)
echo 4. SPI Test (Loopback)
echo 5. All Tests Sequential
echo 6. Cancel
echo.
choice /C 123456 /N /M "Enter your choice (1-6): "

if errorlevel 6 exit /b 0
if errorlevel 5 goto test_all
if errorlevel 4 goto test_spi
if errorlevel 3 goto test_pwm
if errorlevel 2 goto test_adc
if errorlevel 1 goto test_gpio

:test_gpio
echo.
echo Building GPIO Test...
set TEST_MODE=TEST_MODE_GPIO
goto build

:test_adc
echo.
echo Building ADC Test...
set TEST_MODE=TEST_MODE_ADC
goto build

:test_pwm
echo.
echo Building PWM Test...
set TEST_MODE=TEST_MODE_PWM
goto build

:test_spi
echo.
echo Building SPI Test...
set TEST_MODE=TEST_MODE_SPI
goto build

:test_all
echo.
echo Building All Tests...
set TEST_MODE=TEST_MODE_ALL
goto build

:build
echo.
echo ========================================
echo Creating test configuration...
echo ========================================

REM Backup original main.c
if not exist main.c.backup (
    copy main.c main.c.backup >nul
    echo Original main.c backed up
)

REM Copy test main
copy /Y main_test.c main.c >nul
echo Test main.c activated

REM Build
echo.
echo Building project...
cbuild PumpCtrl.csolution.yml

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo Test Build SUCCESS!
    echo ========================================
    echo.
    echo Test Mode: %TEST_MODE%
    echo Binary: out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin
    echo.
    echo To flash to hardware:
    echo   flash.bat
    echo.
    echo To restore normal firmware:
    echo   restore_main.bat
    echo.
) else (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
    echo Restoring original main.c...
    copy /Y main.c.backup main.c >nul
)

pause
