@echo off
echo ========================================
echo Code Explorer - PumpCtrl Project
echo ========================================
echo.

echo What would you like to explore?
echo.
echo 1. View project structure (file tree)
echo 2. Open main entry point (main.c)
echo 3. Open control algorithms (application.c)
echo 4. Open task scheduler (Task.c)
echo 5. Open sensor reading (analog_input.c)
echo 6. Open CAN protocol (can_protocol.c)
echo 7. Open code understanding guide
echo 8. Search for a function or variable
echo 9. Exit
echo.
choice /C 123456789 /N /M "Enter your choice (1-9): "

if errorlevel 9 exit /b 0
if errorlevel 8 goto search
if errorlevel 7 goto guide
if errorlevel 6 goto can
if errorlevel 5 goto sensors
if errorlevel 4 goto task
if errorlevel 3 goto app
if errorlevel 2 goto main
if errorlevel 1 goto structure

:structure
echo.
echo ========================================
echo Project Structure
echo ========================================
echo.
echo PumpCtrl/
echo ├── USER/
echo │   └── main.c                    [Entry point]
echo ├── CORE/
echo │   ├── startup_stm32f40_41xxx.s  [Startup code]
echo │   └── system_stm32f4xx.c        [System init]
echo ├── Function/
echo │   ├── application.c             [Control algorithms] ⭐
echo │   ├── Task.c                    [Task scheduler] ⭐
echo │   ├── analog_input.c            [Sensor reading] ⭐
echo │   ├── algorithm.c               [PID, filters]
echo │   ├── can_protocol.c            [CAN communication]
echo │   ├── flash.c                   [Parameter storage]
echo │   ├── Diagnose.c                [Fault detection]
echo │   ├── limit.c                   [Safety limits]
echo │   ├── CheckTable.c              [Lookup tables]
echo │   └── stm32f4xx_it.c            [Interrupts]
echo ├── HardWare_Init/
echo │   ├── Hw_GPIO.c                 [GPIO setup]
echo │   ├── Hw_ADC.c                  [ADC setup]
echo │   ├── Hw_TIM.c                  [Timer/PWM]
echo │   ├── Hw_SPI.c                  [SPI communication]
echo │   ├── Hw_CAN.c                  [CAN setup]
echo │   └── AD7689.c                  [External ADC]
echo ├── FWLIB/
echo │   └── src/                      [STM32 drivers]
echo └── LIB/
echo     ├── Deadzone_Current.lib      [Precompiled]
echo     └── PID_AREA.lib              [Precompiled]
echo.
echo ⭐ = Most important files to understand
echo.
pause
goto end

:main
echo.
echo Opening main.c...
echo.
echo Key sections to look for:
echo - main() function
echo - Hardware initialization calls
echo - Main while(1) loop
echo - TaskProcess() call
echo.
if exist ..\USER\main.c (
    start notepad ..\USER\main.c
) else if exist main.c (
    start notepad main.c
) else (
    echo File not found! Check path.
)
pause
goto end

:app
echo.
echo Opening application.c...
echo.
echo Key sections to look for:
echo - Control loop functions
echo - PID implementations
echo - State machine logic
echo - Global structures (User_Parameter, InstructionSet)
echo.
if exist ..\Function\application.c (
    start notepad ..\Function\application.c
) else (
    echo File not found! It's in ../Function/ directory
)
pause
goto end

:task
echo.
echo Opening Task.c...
echo.
echo Key sections to look for:
echo - TaskParmInit() - initialization
echo - TaskProcess() - main task loop
echo - Task timing counters
echo - Task scheduling logic
echo.
if exist ..\Function\Task.c (
    start notepad ..\Function\Task.c
) else (
    echo File not found! It's in ../Function/ directory
)
pause
goto end

:sensors
echo.
echo Opening analog_input.c...
echo.
echo Key sections to look for:
echo - Sensor reading functions
echo - ADC conversion
echo - Filtering algorithms
echo - Calibration application
echo.
if exist ..\Function\analog_input.c (
    start notepad ..\Function\analog_input.c
) else (
    echo File not found! It's in ../Function/ directory
)
pause
goto end

:can
echo.
echo Opening can_protocol.c...
echo.
echo Key sections to look for:
echo - CAN message definitions
echo - Message parsing
echo - Command handling
echo - Status transmission
echo.
if exist ..\Function\can_protocol.c (
    start notepad ..\Function\can_protocol.c
) else (
    echo File not found! It's in ../Function/ directory
)
pause
goto end

:guide
echo.
echo Opening Code Understanding Guide...
start CODE_UNDERSTANDING_GUIDE.md
pause
goto end

:search
echo.
echo ========================================
echo Search Code
echo ========================================
echo.
set /p SEARCH_TERM="Enter function or variable name to search: "
echo.
echo Searching for "%SEARCH_TERM%"...
echo.
findstr /s /i /n "%SEARCH_TERM%" ..\*.c ..\*.h *.c *.h 2>nul
if errorlevel 1 (
    echo No results found.
) else (
    echo.
    echo Search complete!
)
echo.
pause
goto end

:end
echo.
echo Would you like to explore something else?
choice /C YN /M "Continue exploring? (Y/N)"
if errorlevel 2 exit /b 0
if errorlevel 1 goto start

:start
cls
goto :eof
