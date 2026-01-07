@echo off
echo ========================================
echo Building PumpCtrl Project
echo ========================================
echo.

cbuild PumpCtrl.csolution.yml

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo Build SUCCESS!
    echo ========================================
    echo.
    echo Converting to binary format...
    fromelf --bin --output out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin out\PumpCtrl\PPQ_Ctrl\PumpCtrl.axf
    echo.
    echo Output files:
    echo   - out\PumpCtrl\PPQ_Ctrl\PumpCtrl.axf
    echo   - out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin
    echo   - out\PumpCtrl\PPQ_Ctrl\PumpCtrl.axf.map
    echo.
    echo To flash the firmware, run: flash.bat
) else (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
)

pause
