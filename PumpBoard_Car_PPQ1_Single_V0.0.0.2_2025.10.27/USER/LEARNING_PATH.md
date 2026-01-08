# PumpCtrl Learning Path - Step by Step

## How to Use This Guide

This guide takes you from zero to understanding the entire codebase in a structured way. Follow the lessons in order, and complete the exercises to reinforce your learning.

---

## 📚 Lesson 1: Project Overview (30 minutes)

### Goal
Understand what the system does and how files are organized.

### Reading
1. Open `CODE_UNDERSTANDING_GUIDE.md` - Read sections 1 & 2
2. Open `CODE_MAP.txt` - Review the execution flow diagram

### Exercise
```
1. Run: explore_code.bat → Option 1 (View structure)
2. Identify these file groups:
   - Hardware drivers (HardWare_Init/)
   - Application logic (Function/)
   - STM32 library (FWLIB/)
3. Answer: What are the 3 main control loops?
```

### Check Your Understanding
- [ ] I can explain what this system controls
- [ ] I know where hardware initialization happens
- [ ] I understand the file organization

---

## 📚 Lesson 2: Program Entry & Initialization (45 minutes)

### Goal
Understand how the program starts and initializes hardware.

### Reading
1. Open `main.c` (use explore_code.bat → Option 2)
2. Read from top to bottom
3. Focus on the `main()` function

### Key Concepts
```c
main() does these things in order:
1. Configure interrupt priorities
2. Initialize GPIO pins
3. Load parameters from flash
4. Setup SysTick timer (100µs)
5. Initialize peripherals (SPI, ADC, PWM, CAN)
6. Enter infinite loop calling TaskProcess()
```

### Exercise
```
1. Find the line that sets up the 100µs timer
   Hint: Look for "SysTickConfig"

2. Find where parameters are loaded from flash
   Hint: Look for "InitUserParaProcess"

3. Find the main loop
   Hint: Look for "while(1)"

4. What function is called inside the main loop?
```

### Hands-On
```
1. Build and flash the normal firmware
2. Set breakpoint at beginning of main()
3. Step through initialization (F10 in Keil)
4. Watch each peripheral initialize
```

### Check Your Understanding
- [ ] I can trace the initialization sequence
- [ ] I know what SysTick does
- [ ] I understand the main loop structure

---

## 📚 Lesson 3: Task Scheduling (1 hour)

### Goal
Understand how tasks are scheduled and executed.

### Reading
1. Open `Task.c` (explore_code.bat → Option 4)
2. Read `CODE_MAP.txt` - "TASK EXECUTION FLOW" section

### Key Concepts
```
Task Timing:
- SysTick interrupt: Every 100µs
- Fast tasks: Every 100µs (every cycle)
- Medium tasks: Every 1ms (every 10 cycles)
- Slow tasks: Every 10ms (every 100 cycles)

This is called "cooperative multitasking"
```

### Exercise
```
1. Find TaskProcess() function in Task.c

2. Identify the three task groups:
   - Fast tasks (100µs)
   - Medium tasks (1ms)
   - Slow tasks (10ms)

3. What counter is used for 1ms tasks?
   Hint: Look for "counter_1ms" or similar

4. List 3 functions called in fast tasks
```

### Hands-On
```
1. Add a counter variable:
   static uint32_t debug_counter = 0;

2. Increment it in TaskProcess():
   debug_counter++;

3. Watch it in debugger
4. Calculate: How fast does it increment?
   Answer: 10,000 times per second (100µs period)
```

### Check Your Understanding
- [ ] I know how often TaskProcess() runs
- [ ] I can identify fast vs slow tasks
- [ ] I understand cooperative multitasking

---

## 📚 Lesson 4: Reading Sensors (1 hour)

### Goal
Understand how sensors are read and processed.

### Reading
1. Open `analog_input.c` (explore_code.bat → Option 5)
2. Read `CODE_UNDERSTANDING_GUIDE.md` - Section 3, "analog_input.c"

### Key Concepts
```
Sensor Reading Pipeline:
1. ADC conversion (hardware)
2. Read raw value (0-4095 for 12-bit)
3. Apply low-pass filter (remove noise)
4. Apply calibration (convert to real units)
5. Check limits (validate range)
```

### Exercise
```
1. Find the function that reads pressure sensor
   Hint: Look for "Pressure" or "Prs" in function names

2. Find where ADC values are read
   Hint: Look for "ADC_GetConversionValue" or similar

3. Find filtering code
   Hint: Look for multiplication and division

4. What is the ADC resolution?
   Answer: 12-bit = 0-4095 range
```

### Hands-On
```
1. Find a sensor reading variable
   Example: g_u32_AngFdb (angle feedback)

2. Add to watch window in debugger

3. Run program and observe value

4. If possible, change sensor input and watch value change
```

### Check Your Understanding
- [ ] I know how ADC values are read
- [ ] I understand filtering purpose
- [ ] I can find sensor variables

---

## 📚 Lesson 5: Control Algorithms (2 hours)

### Goal
Understand PID control and how control loops work.

### Reading
1. Open `application.c` (explore_code.bat → Option 3)
2. Read `CODE_UNDERSTANDING_GUIDE.md` - Section 4A "Control Loop Fundamentals"
3. Read `CODE_MAP.txt` - "CONTROL LOOP DETAIL" section

### Key Concepts
```
PID Control:
- P (Proportional): Immediate response to error
- I (Integral): Eliminates steady-state error
- D (Derivative): Reduces oscillation

Formula:
Output = Kp×Error + Ki×∫Error + Kd×dError/dt

Cascaded Control:
Pressure Loop → Angle Command → Angle Loop → Current Command → PWM
```

### Exercise
```
1. Find the pressure control loop function
   Hint: Look for "Pressure" and "Loop" or "Control"

2. Identify these parts:
   - Error calculation (Target - Actual)
   - PID calculation
   - Output limiting

3. Find the PID gains (Kp, Ki, Kd)
   Hint: Look in User_Parameter structure

4. Trace the data flow:
   Pressure Error → PID → Angle Command → Angle Loop → PWM
```

### Hands-On
```
1. Find pressure setpoint variable
   Example: InstructionSet.g_u16_PerUnitPrsVal

2. Find pressure feedback variable
   Example: DetectorFdbVal.g_u32_PrsFdb

3. Find pressure error variable

4. Watch all three in debugger:
   - Setpoint (what you want)
   - Feedback (what you have)
   - Error (difference)

5. Observe how error changes when setpoint changes
```

### Advanced Exercise
```
Try changing PID gains (carefully!):
1. Find Kp, Ki, Kd values
2. Reduce all by 50% (safer)
3. Observe system response
4. Restore original values

Note: Only do this in simulation or with supervision!
```

### Check Your Understanding
- [ ] I understand PID control basics
- [ ] I can find control loop code
- [ ] I know how cascaded control works
- [ ] I can identify setpoint, feedback, and error

---

## 📚 Lesson 6: PWM Output (45 minutes)

### Goal
Understand how control outputs are generated.

### Reading
1. Open `Hw_TIM.c` from HardWare_Init folder
2. Read about PWM in `DEBUG_GUIDE.md`

### Key Concepts
```
PWM (Pulse Width Modulation):
- Frequency: How fast it switches (e.g., 1kHz)
- Duty Cycle: % of time ON (0-100%)
- Used to control: Solenoid valves, motors

Timer Configuration:
- ARR (Auto-Reload): Sets frequency
- CCR (Capture/Compare): Sets duty cycle
- Duty % = (CCR / ARR) × 100
```

### Exercise
```
1. Find TIM2 initialization function
   Hint: Look for "TIM2_init"

2. Find PWM frequency setting
   Hint: Look for ARR value

3. Find duty cycle variables
   Hint: Look for CCR2, CCR4, or similar

4. Calculate PWM frequency:
   Frequency = Clock / (Prescaler × ARR)
```

### Hands-On
```
1. Find PWM output variables:
   g_u16_TestTim2_CCR2
   g_u16_TestTim2_CCR4

2. Watch in debugger

3. Observe how they change based on control output

4. If you have oscilloscope:
   - Connect to PA1 (TIM2_CH2)
   - Measure frequency and duty cycle
```

### Check Your Understanding
- [ ] I understand PWM basics
- [ ] I know how duty cycle is set
- [ ] I can find PWM output variables

---

## 📚 Lesson 7: CAN Communication (1 hour)

### Goal
Understand how the system communicates via CAN bus.

### Reading
1. Open `can_protocol.c` (explore_code.bat → Option 6)
2. Open `Hw_CAN.c` from HardWare_Init folder

### Key Concepts
```
CAN Bus:
- Multi-device communication
- Message-based (not point-to-point)
- Each message has ID and data (0-8 bytes)
- Devices filter messages by ID

Message Structure:
- ID: Message identifier (e.g., 0x100)
- DLC: Data length (0-8 bytes)
- Data: Payload bytes
```

### Exercise
```
1. Find CAN message IDs
   Hint: Look for #define with MSG_ID or similar

2. Find message receive handler
   Hint: Look for "CAN_ProcessRxMessage" or similar

3. Find message send function
   Hint: Look for "CAN_SendStatus" or similar

4. Identify what data is sent in status messages
```

### Hands-On
```
1. Find CAN receive interrupt handler
   Location: stm32f4xx_it.c
   Function: CAN1_RX0_IRQHandler

2. Set breakpoint there

3. If CAN is connected, trigger a message

4. Step through message processing

5. Watch how data is extracted from message
```

### Check Your Understanding
- [ ] I understand CAN message structure
- [ ] I can find message definitions
- [ ] I know how messages are processed

---

## 📚 Lesson 8: Diagnostics & Safety (1 hour)

### Goal
Understand fault detection and safety mechanisms.

### Reading
1. Open `Diagnose.c` from Function folder
2. Read about watchdog in `main.c`

### Key Concepts
```
Safety Mechanisms:
1. Watchdog Timer: Resets if not fed regularly
2. Sensor Validation: Checks if readings are valid
3. Limit Checking: Prevents dangerous values
4. Fault Flags: Records what went wrong
5. Safe Mode: Reduces output when fault detected
```

### Exercise
```
1. Find watchdog initialization
   Location: main.c
   Function: IWDG_Init()

2. Find watchdog feed function
   Function: IWDG_Feed()

3. Find fault detection code
   Location: Diagnose.c

4. List 5 types of faults that can be detected
```

### Hands-On
```
1. Find fault status variable
   Hint: Look for "fault" or "error" in global variables

2. Watch in debugger

3. Trigger a fault (if safe):
   - Disconnect a sensor
   - Set invalid parameter
   - Simulate timeout

4. Observe fault handling
```

### Check Your Understanding
- [ ] I understand watchdog purpose
- [ ] I know how faults are detected
- [ ] I can find safety limit code

---

## 📚 Lesson 9: Parameter Storage (45 minutes)

### Goal
Understand how parameters are saved and loaded.

### Reading
1. Open `flash.c` from Function folder
2. Read about flash memory in `CODE_MAP.txt`

### Key Concepts
```
Flash Memory:
- Non-volatile (survives power loss)
- Organized in sectors
- Must erase before writing
- Limited write cycles (~10,000)

Parameter Storage:
- PID gains
- Calibration values
- Configuration flags
- Operating limits
```

### Exercise
```
1. Find parameter load function
   Hint: Look for "Load" or "Init" in flash.c

2. Find parameter save function
   Hint: Look for "Save" in flash.c

3. Find flash memory address
   Hint: Look for 0x080xxxxx addresses

4. What happens if flash is empty/corrupted?
   Hint: Look for default values
```

### Check Your Understanding
- [ ] I know where parameters are stored
- [ ] I understand flash limitations
- [ ] I can find save/load functions

---

## 📚 Lesson 10: Putting It All Together (2 hours)

### Goal
Trace complete data flow from sensor to output.

### Exercise: Trace Pressure Control
```
1. Start at sensor reading
   File: analog_input.c
   Find: Pressure sensor read function

2. Follow to control loop
   File: application.c
   Find: Pressure control loop

3. See PID calculation
   File: algorithm.c
   Find: PID_Calculate()

4. Follow to output
   File: Hw_TIM.c
   Find: PWM update

5. Draw the complete flow diagram
```

### Exercise: Modify a Parameter
```
1. Find a PID gain (e.g., pressure Kp)

2. Change its value in code

3. Rebuild and flash

4. Observe system behavior

5. Restore original value

Note: Only do this in safe environment!
```

### Exercise: Add Debug Output
```
1. Add a global counter:
   uint32_t g_debug_loop_count = 0;

2. Increment in TaskProcess():
   g_debug_loop_count++;

3. Add CAN message to send it:
   - Every 100ms
   - Message ID: 0x500
   - Data: counter value

4. Test and verify
```

### Check Your Understanding
- [ ] I can trace complete data flow
- [ ] I can modify and test code
- [ ] I understand system as a whole

---

## 🎓 Graduation Project

### Build a New Feature

Choose one:

**Option A: Add Temperature Monitoring**
```
1. Add ADC channel for temperature sensor
2. Read and filter temperature
3. Add over-temperature protection
4. Send temperature via CAN
5. Store temperature limits in flash
```

**Option B: Add Data Logging**
```
1. Create circular buffer in RAM
2. Log key variables every 10ms
3. Add CAN command to retrieve log
4. Implement log download protocol
```

**Option C: Improve Diagnostics**
```
1. Add sensor range checking
2. Implement timeout detection
3. Create fault history buffer
4. Add fault code definitions
5. Send detailed fault info via CAN
```

---

## 📖 Reference Materials

### Quick Access
```cmd
explore_code.bat           # Interactive code explorer
CODE_UNDERSTANDING_GUIDE.md # Complete reference
CODE_MAP.txt               # Visual diagrams
DEBUG_GUIDE.md             # Debugging help
```

### When You're Stuck
1. Check `CODE_MAP.txt` for visual flow
2. Use `explore_code.bat` to find functions
3. Read relevant section in `CODE_UNDERSTANDING_GUIDE.md`
4. Set breakpoints and use debugger
5. Search code for similar examples

---

## ✅ Completion Checklist

### Beginner Level
- [ ] Completed Lessons 1-3
- [ ] Can explain program flow
- [ ] Can navigate codebase
- [ ] Can use debugger basics

### Intermediate Level
- [ ] Completed Lessons 4-7
- [ ] Understand control loops
- [ ] Can trace data flow
- [ ] Can modify parameters

### Advanced Level
- [ ] Completed Lessons 8-10
- [ ] Understand entire system
- [ ] Can add new features
- [ ] Completed graduation project

---

## 🚀 Next Steps After Completion

1. **Optimize Performance**
   - Profile execution time
   - Optimize critical loops
   - Reduce interrupt latency

2. **Add Features**
   - Implement your graduation project
   - Add advanced diagnostics
   - Improve user interface

3. **Study Advanced Topics**
   - Real-time operating systems (RTOS)
   - Advanced control theory
   - Communication protocols

4. **Share Knowledge**
   - Document your changes
   - Help others learn
   - Contribute improvements

---

**Congratulations on starting your learning journey!**

Begin with Lesson 1 and work your way through. Take your time, do the exercises, and don't hesitate to revisit earlier lessons.

Good luck! 🎉
