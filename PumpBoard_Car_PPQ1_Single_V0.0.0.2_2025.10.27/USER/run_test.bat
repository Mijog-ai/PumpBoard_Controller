@echo off
echo ========================================
echo STM32F407ZE Hardware Test Runner
echo ========================================
echo.

echo Current test firmware is built and ready!
echo Binary: out\PumpCtrl\PPQ_Ctrl\PumpCtrl_TEST.bin
echo.
echo Current Test: GPIO LED Blink Test
echo Expected: LEDs on PF9 and PF10 blink alternately
echo.
echo ========================================
echo.
echo What would you like to do?
echo.
echo 1. Flash test firmware to hardware
echo 2. Change test mode and rebuild
echo 3. Restore normal firmware
echo 4. View test guide
echo 5. Exit
echo.
choice /C 12345 /N /M "Enter your choice (1-5): "

if errorlevel 5 exit /b 0
if errorlevel 4 goto guide
if errorlevel 3 goto restore
if errorlevel 2 goto change_test
if errorlevel 1 goto flash

:flash
echo.
echo Flashing test firmware...
flash.bat
goto end

:change_test
echo.
echo ========================================
echo Select Test Mode
echo ========================================
echo.
echo 1. GPIO Test (LED Blink)
echo 2. ADC Test (Analog Input)
echo 3. PWM Test (Timer Output)
echo 4. SPI Test (Loopback)
echo 5. All Tests Sequential
echo 6. Cancel
echo.
choice /C 123456 /N /M "Enter test mode (1-6): "

if errorlevel 6 goto end
if errorlevel 5 set TEST_DEF=TEST_MODE_ALL
if errorlevel 4 set TEST_DEF=TEST_MODE_SPI
if errorlevel 3 set TEST_DEF=TEST_MODE_PWM
if errorlevel 2 set TEST_DEF=TEST_MODE_ADC
if errorlevel 1 set TEST_DEF=TEST_MODE_GPIO

echo.
echo Updating test mode to %TEST_DEF%...
echo.

REM Update main_test.c to enable selected test
powershell -Command "(Get-Content main_test.c) -replace '#define TEST_MODE_\w+\s+1', '// #define TEST_MODE_XXX 1' | Set-Content main_test.c.tmp"
powershell -Command "(Get-Content main_test.c.tmp) -replace '// #define %TEST_DEF%\s+1', '#define %TEST_DEF% 1' | Set-Content main_test.c"
del main_test.c.tmp

copy /Y main_test.c main.c >nul

echo Rebuilding firmware...
cbuild PumpCtrl.csolution.yml --rebuild

if %errorlevel% equ 0 (
    fromelf --bin --output out\PumpCtrl\PPQ_Ctrl\PumpCtrl_TEST.bin out\PumpCtrl\PPQ_Ctrl\PumpCtrl.axf
    echo.
    echo ========================================
    echo Test firmware rebuilt successfully!
    echo ========================================
    echo.
    echo Run this script again to flash to hardware.
) else (
    echo.
    echo Build failed! Check errors above.
)
goto end

:restore
echo.
restore_main.bat
goto end

:guide
echo.
echo Opening test guide...
start RUN_HARDWARE_TEST.md
goto end

:end
pause
