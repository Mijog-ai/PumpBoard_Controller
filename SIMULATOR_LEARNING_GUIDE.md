# PumpBoard Controller - Simulator Learning Guide

## Overview
Your PumpBoard Controller is a **closed-loop control system** for a hydraulic pump with:
- **Angle Control** - Controls pump swash plate tilt angle
- **Pressure Control** - Maintains hydraulic pressure
- **Current Control** - Manages solenoid valve currents
- **Power Management** - Monitors and controls power consumption

This guide will teach you how to understand and debug the code using Keil simulator.

---

## System Architecture Quick Reference

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Control Flow                        │
├─────────────────────────────────────────────────────────────┤
│ 1. Initialize Hardware (GPIO, ADC, SPI, TIM2, PWM)         │
│ 2. Load User Parameters from Flash                          │
│ 3. Start SysTick Timer (100us interrupts)                  │
│ 4. Enter main loop: TaskProcess()                           │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│  Sensor Inputs   │────▶│  Control Loops   │────▶│   PWM Outputs    │
├──────────────────┤     ├──────────────────┤     ├──────────────────┤
│ - Angle Sensor   │     │ - Angle PID      │     │ - Solenoid A PWM │
│ - Pressure Sensor│     │ - Pressure PID   │     │ - Solenoid B PWM │
│ - Current Sensors│     │ - Current PID    │     │ - Valve Position │
└──────────────────┘     └──────────────────┘     └──────────────────┘
```

---

## Part 1: What to Check in Simulator

### A. Start with System Initialization (main.c)

**Key checkpoints:**

1. **Before starting debug, locate these in code:**
   - `main.c:75` - main() function entry
   - `main.c:86` - GPIO_Init_Func() - I/O pin configuration
   - `main.c:90` - InitUserParaProcess() - Load saved parameters
   - `main.c:92` - SysTickConfig() - System timer setup (100us tick)
   - `main.c:96` - TIM2_init() - PWM timer for solenoids

2. **Set breakpoints at:**
   ```
   main.c:75   - Start of main()
   main.c:113  - Simulation test code section
   main.c:138  - while(1) loop
   ```

3. **What to observe:**
   - Check that g_u8_SimulationMode = 1 (enables simulation)
   - Verify control loop enable flags are set
   - Note initial reference values (TiltAngRef, Cur_A_Ref, etc.)

---

### B. Understanding the Task Scheduler (Task.c)

Your system uses **time-sliced task scheduling**:

| Task | Period | What it does |
|------|--------|--------------|
| Task1ms_v_g()  | 1ms   | Fast control loops (currently empty) |
| Task5ms_v_g()  | 5ms   | Updates command data, saves parameters |
| Task10ms_v_g() | 10ms  | (currently empty) |
| Task100ms_v_g()| 100ms | (currently empty) |

**How to trace task execution:**

1. **Set breakpoint in:**
   ```
   Task.c:95 - TaskProcess() function
   ```

2. **Watch variables:**
   ```
   u32_TimeTick_1ms   - Counts in 1ms increments
   u32_TimeTick_5ms   - Counts in 5ms increments
   TaskComps[]        - Task control structure array
   ```

3. **Step through the loop:**
   - Press F10 (Step Over) to see which tasks execute
   - Notice how tasks only run when their timer expires
   - Each task resets its timer after execution

**Exercise:**
- Add a counter variable in Task5ms_v_g()
- Watch it increment every 5ms in the simulator
- This proves the timing mechanism works

---

### C. Control Loop Deep Dive

#### 1. Angle Control Loop

**Purpose:** Controls the pump swash plate angle (0-100% tilt)

**Key variables to watch:**
```
InstructionSet.TiltAngRef           - Desired angle (0-10000 = 0-100%)
DetectorFdbVal.g_u32_AngFdb         - Actual angle from sensor
Angle_Loop.error                     - Control error (ref - feedback)
Angle_Loop.output                    - PID controller output
```

**How to test in simulator:**

1. **Open Watch Window 1** (already configured in .uvoptx)
   - Look for "InstructionSet.TiltAngRef"
   - You'll see it set to 5000 (50% angle)

2. **Open Watch Window 2**
   - Shows DetectorFdbVal.g_u32_AngFdb = 2000 (20% actual)
   - **This creates a 30% error!**

3. **Set breakpoint at:**
   - Search for "Angle_Loop" PID calculation function
   - Step through to see how error drives the output

4. **Modify values while running:**
   - Pause execution
   - In Watch Window, change InstructionSet.TiltAngRef to 8000
   - Resume and observe error change

**What you should learn:**
- How error = reference - feedback
- How PID tries to minimize error
- How output changes to reduce error

---

#### 2. Pressure Control Loop

**Purpose:** Maintains hydraulic system pressure

**Key variables:**
```
InstructionSet.g_u16_PerUnitPrsVal  - Desired pressure (per-unit value)
DetectorFdbVal.g_u32_PrsFdb         - Actual pressure feedback
Pressure_Loop.error                  - Pressure error
Pressure_Loop.output                 - PID output for pressure
```

**How to observe pressure control:**

1. **Add to Watch Window:**
   ```
   InstructionSet.g_u16_PerUnitPrsVal
   DetectorFdbVal.g_u32_PrsFdb
   Pressure_Loop
   ```

2. **Create test scenario:**
   - Set desired pressure to 3000
   - Set feedback to 1000
   - Watch PID response over time

---

#### 3. Current Control Loops (Solenoid A & B)

**Purpose:** Controls current through solenoid valves (drives pump angle)

**Key variables:**
```
InstructionSet.Cur_A_Ref             - Desired current A (mA)
InstructionSet.Cur_B_Ref             - Desired current B (mA)
DetectorFdbVal.g_s32_CurFdb_A        - Actual current A
DetectorFdbVal.g_s32_CurFdb_B        - Actual current B
g_u32_PWMOutput_A                    - PWM duty cycle for solenoid A
g_u32_PWMOutput_B                    - PWM duty cycle for solenoid B
```

**Already in Watch Window 1!** You can see:
- Current references (1500mA each)
- Current feedback (800mA - creating error)
- PWM outputs

---

### D. PWM Output Analysis

**What PWM controls:**
- Solenoid valve current → Controls pump angle
- Higher PWM duty cycle = More current = Larger angle

**How to check PWM:**

1. **Add to Watch Window:**
   ```
   g_u32_PWMOutput_A     - PWM value for solenoid A (0-21000)
   g_u32_PWMOutput_B     - PWM value for solenoid B
   TIM2->CCR2            - Timer 2 compare register (actual HW PWM)
   TIM2->CCR4            - Timer 4 compare register
   ```

2. **Calculate duty cycle:**
   ```
   Duty Cycle % = (PWMOutput / PWM_ARR) * 100
   Where PWM_ARR = 21000 (from main.c:28)

   Example: PWMOutput = 10500 → 50% duty cycle
   ```

3. **Test PWM response:**
   - Increase Cur_A_Ref from 1500 to 2500
   - Watch g_u32_PWMOutput_A increase
   - This is the PID trying to increase current

---

## Part 2: Step-by-Step Learning Process

### Week 1: Basic Understanding

#### Day 1: System Initialization
**Goal:** Understand startup sequence

**Tasks:**
1. Open main.c in editor
2. Set breakpoint at line 75 (main entry)
3. Start debug (F5)
4. Step through each initialization function (F10)
5. **Document what each init function does**

**Questions to answer:**
- What clock speed is the MCU configured for?
- How many PWM channels are initialized?
- What is the PWM frequency?

---

#### Day 2: Task Scheduler
**Goal:** Understand timing mechanism

**Tasks:**
1. Set breakpoint in TaskProcess() (Task.c:95)
2. Add u32_TimeTick_1ms to Watch Window
3. Run continuously (F5)
4. Pause periodically and check tick count
5. Calculate actual time passed

**Exercise:**
- Modify Task5ms_v_g() to toggle a variable
- Verify it happens every 5ms

---

#### Day 3: Watch Variables
**Goal:** Learn to observe key variables

**Tasks:**
1. Open Debug → View → Watch Windows
2. Add these variables:
   ```
   InstructionSet
   DetectorFdbVal
   Angle_Loop
   Pressure_Loop
   g_u32_PWMOutput_A
   g_u32_PWMOutput_B
   ```
3. Run simulator and watch values change
4. Pause and modify values manually
5. Resume and observe system response

---

### Week 2: Control Loop Analysis

#### Day 4-5: Angle Control Deep Dive
**Goal:** Master angle control loop

**Tasks:**
1. Find the Angle PID calculation function
2. Set breakpoint inside the PID loop
3. **Step through ONE complete PID cycle:**
   - Note: error = reference - feedback
   - See: P-term = Kp * error
   - See: I-term accumulates over time
   - See: D-term = derivative of error
   - Output = P + I + D

4. **Create a table:**
   ```
   Cycle | AngRef | AngFdb | Error | P-term | I-term | D-term | Output
   -------|--------|--------|-------|--------|--------|--------|--------
   1      |        |        |       |        |        |        |
   2      |        |        |       |        |        |        |
   ```

5. Run 10 cycles and fill the table
6. **Plot by hand:** Error vs. Time

**Questions:**
- Does error decrease over time?
- What happens if you increase Kp?
- What happens if you set feedback = reference?

---

#### Day 6-7: Pressure & Current Control
**Goal:** Understand cascaded control

**Repeat Day 4-5 tasks for:**
- Pressure loop
- Current loop A
- Current loop B

**Advanced:** Try to understand the relationship:
```
Desired Pressure → Angle Reference → Current Reference → PWM Output
```

---

### Week 3: System Behavior

#### Day 8: Steady State Analysis
**Goal:** Observe system at equilibrium

**Tasks:**
1. Set ref = feedback for all loops
2. Run for 1000 cycles
3. Verify outputs are stable
4. Check that I-term doesn't wind up

---

#### Day 9: Step Response
**Goal:** Characterize transient behavior

**Tasks:**
1. Start with ref = feedback (equilibrium)
2. Suddenly change InstructionSet.TiltAngRef from 5000 → 8000
3. Record error every cycle for 100 cycles
4. **Calculate:**
   - Rise time (time to reach 90% of new setpoint)
   - Settling time (time to reach ±5% of setpoint)
   - Overshoot (max value beyond setpoint)

---

#### Day 10: Disturbance Rejection
**Goal:** Test robustness

**Tasks:**
1. Run system at steady state
2. Suddenly change feedback (simulate sensor disturbance)
3. Watch how PID corrects
4. Time how long it takes to recover

---

## Part 3: Advanced Debugging Techniques

### Using Memory Windows

**View peripheral registers:**
1. Debug → View → Memory Windows → Memory 1
2. Enter addresses:
   ```
   0x40023800  - RCC (clock configuration)
   0x40020000  - GPIOA
   0x40000000  - TIM2 (PWM timer)
   ```

3. Watch registers change in real-time

---

### Using Logic Analyzer View

**Visualize PWM signals:**
1. Debug → View → Logic Analyzer
2. Add variables:
   ```
   g_u32_PWMOutput_A
   g_u32_PWMOutput_B
   ```
3. Run simulator
4. See waveforms over time!

---

### Using Serial Window

**Monitor printf output:**
1. In your code, add printf statements:
   ```c
   printf("Angle Error: %d\n", Angle_Loop.error);
   ```

2. View → Serial Windows → UART #1
3. See debug messages in real-time

**Note:** Requires ITM (Instrumentation Trace Macrocell) setup

---

## Part 4: Common Analysis Tasks

### Task 1: Find Where a Variable is Modified

**Example:** Find all places that modify `InstructionSet.TiltAngRef`

**Method 1: Using Search**
1. Edit → Find in Files (Ctrl+Shift+F)
2. Search for: `TiltAngRef`
3. Look for assignment operators `=`

**Method 2: Using Breakpoint**
1. Right-click variable in Watch Window
2. Break on Write
3. Run code - it will pause when variable changes

---

### Task 2: Trace Function Call Tree

**Example:** What calls TaskProcess()?

**Method:**
1. Right-click function name
2. Find All References
3. Or use Call Stack window during debug

---

### Task 3: Measure Code Execution Time

**Example:** How long does PID calculation take?

**Method:**
```c
volatile uint32_t start_time, end_time, elapsed;

start_time = SysTick->VAL;  // Read SysTick counter
// ... code to measure ...
end_time = SysTick->VAL;

elapsed = start_time - end_time;  // Ticks
// Convert to microseconds based on SysTick config
```

---

### Task 4: Inject Faults for Testing

**Example:** Simulate sensor failure

**Method:**
```c
// In simulator mode, force bad sensor reading
if (g_u8_SimulationMode) {
    DetectorFdbVal.g_u32_AngFdb = 0xFFFF;  // Invalid reading
}
```

Watch how system responds - does it catch the error?

---

## Part 5: Learning Exercises

### Exercise 1: Build a State Machine Observer

**Goal:** Understand system states

**Task:**
Create a document with these sections:
- **INIT State:** What happens during initialization?
- **IDLE State:** System running but no commands
- **RUNNING State:** Control loops active
- **FAULT State:** What triggers faults?

Find code for each state transition.

---

### Exercise 2: Parameter Tuning

**Goal:** Learn PID tuning

**Task:**
1. Find PID coefficients (Kp, Ki, Kd)
2. Record baseline performance (settling time, overshoot)
3. Increase Kp by 20%, test again
4. Increase Ki by 20%, test again
5. Increase Kd by 20%, test again
6. Document which changes helped vs. hurt

---

### Exercise 3: Create Your Own Test

**Goal:** Test angle limits

**Task:**
1. Find the max allowed angle (should be 10000)
2. Try setting reference to 15000 (over limit)
3. What happens? Is there protection?
4. Find the code that enforces limits
5. Test minimum limit (0 or negative)

---

### Exercise 4: Data Flow Diagram

**Goal:** Visualize information flow

**Task:**
Draw a diagram showing:
```
Sensor → ADC → Filter → Error Calculation → PID → PWM → Actuator
```

Label each box with:
- Function name
- File name
- Key variables
- Update rate

---

## Part 6: Things to Learn & Understand

### Fundamental Concepts

1. **Feedback Control**
   - What is a setpoint (reference)?
   - What is feedback?
   - What is error?
   - Why do we need feedback? (Try open-loop: remove feedback, see what happens)

2. **PID Controller**
   - **P (Proportional):** Error × Kp
     - Larger error → Larger output
     - Fast response, but may overshoot

   - **I (Integral):** Sum of all past errors × Ki
     - Eliminates steady-state error
     - Can cause windup (accumulate too much)

   - **D (Derivative):** Rate of error change × Kd
     - Predicts future error
     - Reduces overshoot
     - Sensitive to noise

3. **PWM (Pulse Width Modulation)**
   - Digital output that simulates analog voltage
   - Duty Cycle = ON time / Total time
   - Higher duty cycle = More power to actuator

4. **Filtering**
   - Raw sensor data is noisy
   - Low-pass filter smooths signal
   - Trade-off: smoothness vs. delay

---

### Code Architecture Concepts

1. **Task Scheduling**
   - Why different update rates?
   - Fast loops: stability (current control)
   - Slow loops: efficiency (parameter save)

2. **State Machines**
   - System has discrete states
   - Transitions based on conditions
   - Prevents invalid operations

3. **Interrupt Service Routines (ISRs)**
   - SysTick_Handler() - Called every 100us
   - Increments time counters
   - Triggers tasks

4. **Memory Organization**
   - Flash: Program code + constants
   - SRAM: Variables, stack
   - Registers: Peripheral control
   - Watch how variables are stored

---

### Hardware Concepts (Even in Simulator)

1. **Peripherals**
   - GPIO: Digital I/O pins
   - ADC: Analog-to-Digital Converter (sensors)
   - TIM: Timer for PWM generation
   - SPI: Serial communication to external ADC

2. **Registers**
   - Configuration registers set peripheral behavior
   - Data registers hold input/output values
   - Example: TIM2->CCR2 = PWM duty cycle

3. **Clock System**
   - System clock (168 MHz for STM32F4)
   - Peripheral clocks (84 MHz, 42 MHz)
   - Affects timing calculations

---

## Part 7: Troubleshooting Guide

### Problem: Variables don't change

**Possible causes:**
1. Optimization level too high
   - Solution: Project → Options → C/C++ → Optimization → Level 0
2. Variable not in scope
   - Solution: Make variable global or volatile
3. Code not executing
   - Solution: Set breakpoint, verify code runs

---

### Problem: Can't see structure members

**Solution:**
1. Expand structure in Watch Window (click +)
2. Add specific member: `InstructionSet.TiltAngRef`
3. Check that structure is defined with debug info

---

### Problem: Simulator is too slow

**Solution:**
1. Disable unused peripherals
2. Remove unnecessary watch variables
3. Use "Run" instead of "Step" for long loops
4. Increase simulator speed: Debug → Run → Max Speed

---

### Problem: Values don't match expectations

**Solution:**
1. Check units (mA vs. A, per-unit vs. raw ADC)
2. Verify scaling factors
3. Check for integer overflow
4. Use calculator to manually verify math

---

## Part 8: Resources for Further Learning

### Within the Code

1. **application.h** - All #defines, constants, structures
2. **PID.h** - PID algorithm implementation
3. **algorithm.c** - Utility functions
4. **flash.c** - Parameter save/load

### External Resources

1. **PID Control:**
   - Search: "PID tuning Ziegler-Nichols method"
   - Watch: "PID controller explained" videos

2. **STM32F4:**
   - Download: STM32F407 Reference Manual (RM0090)
   - Read: Timer chapter for PWM understanding

3. **Keil µVision:**
   - Help → uVision Help
   - Sections on debugging, simulation, watch windows

---

## Part 9: Your First Debugging Session (Step-by-Step)

### Session 1: Observe One Control Cycle

**Time needed:** 30 minutes

**Steps:**

1. **Launch Keil and open project**
   ```
   File → Open Project
   Navigate to: USER/PumpCtrl.uvprojx
   ```

2. **Build project**
   ```
   Project → Build Target (F7)
   Check for 0 errors, 0 warnings
   ```

3. **Start debug**
   ```
   Debug → Start/Stop Debug Session (Ctrl+F5)
   Simulator loads with debug.ini configuration
   ```

4. **Open Watch Window 2**
   ```
   View → Watch Windows → Watch 2
   You should see: User_Parameter, DetectorFdbVal, InstructionSet, etc.
   ```

5. **Set breakpoint in main loop**
   ```
   Open main.c
   Find line 140: TaskProcess();
   Click in the left margin to set breakpoint (or press F9)
   Red dot appears
   ```

6. **Run to breakpoint**
   ```
   Press F5 (Run)
   Program stops at breakpoint
   ```

7. **Observe current values**
   ```
   In Watch Window 2, note:
   - InstructionSet.TiltAngRef = ?
   - DetectorFdbVal.g_u32_AngFdb = ?
   - Calculate error = ref - feedback
   ```

8. **Step into TaskProcess()**
   ```
   Press F11 (Step Into)
   You enter TaskProcess() function
   ```

9. **Step through the loop**
   ```
   Press F10 multiple times (Step Over)
   Watch the for() loop iterate
   When Run = 1, task executes
   ```

10. **Continue to next breakpoint**
    ```
    Press F5
    Loop continues
    Repeat to see multiple cycles
    ```

11. **Stop debug session**
    ```
    Debug → Start/Stop Debug Session (Ctrl+F5)
    ```

**Congratulations!** You've traced one complete control cycle!

---

## Part 10: Weekly Goals

### Week 1: Familiarization
- [ ] Successfully start simulator without errors
- [ ] Navigate code in editor
- [ ] Set and hit breakpoints
- [ ] View variables in Watch Windows
- [ ] Understand main() flow

### Week 2: Control Loops
- [ ] Trace angle control from reference to output
- [ ] Trace pressure control
- [ ] Trace current control
- [ ] Understand PID calculation
- [ ] Modify references and observe response

### Week 3: System Behavior
- [ ] Test steady-state operation
- [ ] Test step response
- [ ] Measure settling time
- [ ] Identify state transitions
- [ ] Document system timing

### Week 4: Deep Dive
- [ ] Understand interrupt timing
- [ ] Measure function execution time
- [ ] Analyze PWM generation
- [ ] Read peripheral registers
- [ ] Create your own test scenarios

---

## Quick Reference Card

**Essential Hotkeys:**
- `F5` - Run / Continue
- `F10` - Step Over
- `F11` - Step Into
- `Shift+F11` - Step Out
- `F9` - Toggle Breakpoint
- `Ctrl+F5` - Start/Stop Debug
- `F7` - Build Project

**Essential Variables:**
```c
// Set references (what you want)
InstructionSet.TiltAngRef       // Angle setpoint (0-10000)
InstructionSet.Cur_A_Ref        // Current A setpoint (mA)
InstructionSet.Cur_B_Ref        // Current B setpoint (mA)

// Read feedback (what you have)
DetectorFdbVal.g_u32_AngFdb     // Angle feedback
DetectorFdbVal.g_s32_CurFdb_A   // Current A feedback
DetectorFdbVal.g_s32_CurFdb_B   // Current B feedback

// Check outputs (what system is doing)
g_u32_PWMOutput_A               // PWM to solenoid A
g_u32_PWMOutput_B               // PWM to solenoid B

// Monitor loops
Angle_Loop.error                // Angle error
Angle_Loop.output               // Angle PID output
Pressure_Loop.error             // Pressure error
Pressure_Loop.output            // Pressure PID output
```

**Key Files:**
- `USER/main.c` - Program entry, initialization, main loop
- `Function/Task.c` - Task scheduler
- `Function/application.c` - Control algorithms
- `Function/stm32f4xx_it.c` - Interrupt handlers
- `USER/debug.ini` - Simulator memory map (YOU FIXED THIS!)

---

## Final Tips

1. **Start simple** - Don't try to understand everything at once
2. **Use paper** - Draw diagrams, write notes
3. **Ask "why?"** - Why is this variable here? Why this timing?
4. **Change things** - Best way to learn is to break things (in simulator!)
5. **Be patient** - Embedded systems are complex, take time
6. **Document** - Keep a learning journal

**Remember:** The simulator is your safe playground. You can't damage hardware, so experiment freely!

---

## Next Steps

1. Read this guide once completely
2. Try "Session 1" from Part 9
3. Pick one exercise from Part 5
4. Document what you learned
5. Come back with specific questions

Good luck with your learning journey! 🚀
