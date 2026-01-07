@echo off
echo ========================================
echo Restore Original Firmware
echo ========================================
echo.

if not exist main.c.backup (
    echo ERROR: No backup found!
    echo Nothing to restore.
    pause
    exit /b 1
)

echo Restoring original main.c...
copy /Y main.c.backup main.c >nul

echo.
echo Rebuilding normal firmware...
cbuild PumpCtrl.csolution.yml

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo Restore SUCCESS!
    echo ========================================
    echo.
    echo Normal firmware restored and built.
    echo You can now flash the normal application.
    echo.
) else (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
    echo main.c was restored but build failed.
    echo Check for compilation errors.
)

pause
