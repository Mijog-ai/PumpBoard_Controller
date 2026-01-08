# PumpCtrl Code Understanding Guide

## Table of Contents
1. [Project Overview](#project-overview)
2. [Architecture & Data Flow](#architecture--data-flow)
3. [Module Breakdown](#module-breakdown)
4. [Key Concepts](#key-concepts)
5. [Code Reading Path](#code-reading-path)
6. [Common Patterns](#common-patterns)

---

## 1. Project Overview

### What This System Does
This is a **pump control system** for an STM32F407ZE microcontroller that:
- Controls a hydraulic/pneumatic pump with variable displacement
- Monitors pressure, current, and angle sensors
- Implements closed-loop PID control
- Communicates via CAN bus
- Stores parameters in flash memory

### System Type
**Real-time embedded control system** with:
- 100µs (10kHz) main timing loop
- Multiple control loops (pressure, angle, power)
- Safety features (watchdog, diagnostics)
- External communication (CAN)

---

## 2. Architecture & Data Flow

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         MAIN.C                              │
│                    (Entry Point)                            │
└────────────┬────────────────────────────────────────────────┘
             │
             ├─► Hardware Initialization
             │   ├─► GPIO (Hw_GPIO.c)
             │   ├─► ADC (Hw_ADC.c, AD7689.c)
             │   ├─► PWM/Timers (Hw_TIM.c)
             │   ├─► SPI (Hw_SPI.c)
             │   └─► CAN (Hw_CAN.c)
             │
             ├─► Application Setup
             │   ├─► Load Parameters (flash.c)
             │   ├─► Initialize Tasks (Task.c)
             │   └─► Configure Sensors (analog_input.c)
             │
             └─► Main Loop (while(1))
                 └─► TaskProcess() ◄─── Called every 100µs
                     │
                     ├─► Read Sensors (analog_input.c)
                     │   ├─► ADC channels
                     │   ├─► AD7689 external ADC
                     │   └─► CAN messages
                     │
                     ├─► Run Control Algorithms (application.c)
                     │   ├─► Pressure Control Loop
                     │   ├─► Angle Control Loop
                     │   ├─► Power Control Loop
                     │   └─► Leak Detection
                     │
                     ├─► Apply Limits (limit.c)
                     │   ├─► Current limits
                     │   ├─► Pressure limits
                     │   └─► Angle limits
                     │
                     ├─► Update Outputs (Hw_TIM.c)
                     │   └─► PWM duty cycles
                     │
                     ├─► Diagnostics (Diagnose.c)
                     │   ├─► Fault detection
                     │   └─► Error handling
                     │
                     └─► CAN Communication (can_protocol.c)
                         ├─► Send status
                         └─► Receive commands
```

### Data Flow

```
Sensors → ADC → Filtering → Control Algorithm → Limits → PWM → Actuators
   ↓                                                              ↑
   └──────────────► CAN Bus ◄──────────────────────────────────┘
                  (Commands & Status)
```

---

## 3. Module Breakdown

### Layer 1: Hardware Abstraction (HardWare_Init/)

#### Hw_GPIO.c
**Purpose**: Configure all GPIO pins
```c
// What it does:
- Sets up input/output pins
- Configures alternate functions
- Enables pull-ups/pull-downs

// Key functions:
GPIO_Init_Func()        // Initialize all GPIO
```

#### Hw_ADC.c
**Purpose**: Internal ADC configuration
```c
// What it does:
- Configures ADC1 for analog inputs
- Sets up DMA for continuous conversion
- Manages multiple channels

// Key functions:
Adc1_Analog_Input_Init()  // Setup ADC
ADC_Read_Channel(ch)      // Read specific channel
```

#### AD7689.c
**Purpose**: External 16-bit ADC via SPI
```c
// What it does:
- High-precision analog measurements
- SPI communication with AD7689 chip
- Used for critical sensor readings

// Key functions:
AD7689InitFunc()          // Initialize external ADC
AD7689_Read()             // Read ADC value
```

#### Hw_TIM.c
**Purpose**: Timer/PWM configuration
```c
// What it does:
- Generates PWM signals for pump control
- Configures timer interrupts
- Sets duty cycles

// Key functions:
TIM2_init()               // Initialize timer
TIM_SetDutyCycle()        // Update PWM output
```

#### Hw_SPI.c
**Purpose**: SPI bus communication
```c
// What it does:
- SPI3 for AD7689 communication
- Master mode configuration
- Data transfer functions

// Key functions:
SPI3_Int()                // Initialize SPI
SPI_TransferByte()        // Send/receive data
```

#### Hw_CAN.c
**Purpose**: CAN bus communication
```c
// What it does:
- CAN1 configuration
- Message filtering
- Transmit/receive functions

// Key functions:
CAN_Init()                // Setup CAN bus
CAN_SendMessage()         // Transmit
CAN_ReceiveMessage()      // Receive
```

---

### Layer 2: Application Functions (Function/)

#### application.c ⭐ CORE LOGIC
**Purpose**: Main control algorithms
```c
// What it does:
- Implements PID control loops
- Manages pump state machine
- Coordinates all control functions

// Key structures:
User_Parameter              // System configuration
InstructionSet              // Setpoints/commands
DetectorFdbVal              // Sensor feedback

// Key functions:
PressureControlLoop()       // Pressure PID
AngleControlLoop()          // Angle/displacement PID
PowerControlLoop()          // Power limiting
LeakDetection()             // Fault detection
```

**Control Loops Explained:**

1. **Pressure Control**
   ```
   Target Pressure → PID → Angle Command → PWM → Pump → Actual Pressure
                      ↑                                        ↓
                      └────────── Feedback ──────────────────┘
   ```

2. **Angle Control**
   ```
   Angle Command → PID → Current Command → PWM → Solenoid → Swash Plate Angle
                    ↑                                              ↓
                    └──────────── Feedback ────────────────────────┘
   ```

3. **Power Control**
   ```
   Monitors: Current × Voltage
   Limits: Maximum power consumption
   Action: Reduces angle if power too high
   ```

#### Task.c
**Purpose**: Task scheduler
```c
// What it does:
- Manages periodic tasks
- Timing control
- Task prioritization

// Key functions:
TaskParmInit()              // Initialize tasks
TaskProcess()               // Main task loop (called every 100µs)
```

**Task Timing:**
```
SysTick: 100µs
├─► Fast Loop (100µs)
│   ├─► Read sensors
│   ├─► Run control loops
│   └─► Update outputs
│
├─► Medium Loop (1ms)
│   ├─► CAN communication
│   └─► Diagnostics
│
└─► Slow Loop (10ms)
    ├─► Parameter updates
    └─► Flash operations
```

#### analog_input.c
**Purpose**: Sensor reading and processing
```c
// What it does:
- Reads all analog sensors
- Applies calibration
- Filters noise
- Converts to engineering units

// Key functions:
ReadAllSensors()            // Read all inputs
FilterSensorData()          // Apply filters
ConvertToUnits()            // Scale to real values

// Sensors:
- Pressure sensor (0-300 bar)
- Current sensors A & B (0-5A)
- Angle sensor (0-100%)
- Temperature sensors
```

#### algorithm.c
**Purpose**: Control algorithms and math
```c
// What it does:
- PID algorithm implementation
- Low-pass filters
- Deadband compensation
- Ramp generators

// Key functions:
PID_Calculate()             // PID computation
LowPassFilter()             // Noise filtering
RampGenerator()             // Smooth transitions
DeadbandCompensation()      // Eliminate deadzone
```

**PID Algorithm:**
```c
// Standard PID formula
output = Kp * error + Ki * integral + Kd * derivative

// With anti-windup:
if (output > max) {
    output = max;
    integral = integral_previous;  // Don't accumulate
}
```

#### limit.c
**Purpose**: Safety limits and bounds
```c
// What it does:
- Enforces min/max limits
- Rate limiting (slew rate)
- Safety bounds checking

// Key functions:
ApplyLimits()               // Clamp values
RateLimit()                 // Limit change rate
SafetyCheck()               // Verify bounds
```

#### Diagnose.c
**Purpose**: Fault detection and handling
```c
// What it does:
- Monitors system health
- Detects faults
- Triggers safe mode
- Logs errors

// Fault types:
- Sensor failures
- Over-current
- Over-pressure
- Communication timeout
- Watchdog reset

// Key functions:
DiagnoseProcess()           // Run diagnostics
FaultHandler()              // Handle errors
GetFaultStatus()            // Read fault codes
```

#### flash.c
**Purpose**: Non-volatile parameter storage
```c
// What it does:
- Saves parameters to flash
- Loads parameters on startup
- Validates data integrity
- Factory reset

// Key functions:
InitUserParaProcess()       // Load parameters
SaveParametersToFlash()     // Store parameters
RestoreDefaults()           // Factory reset

// Stored data:
- PID gains
- Calibration values
- Operating limits
- Configuration flags
```

#### can_protocol.c
**Purpose**: CAN communication protocol
```c
// What it does:
- Defines CAN message format
- Handles incoming commands
- Sends status updates
- Protocol implementation

// Message types:
0x100: Command message
0x200: Status message
0x300: Parameter read/write
0x400: Diagnostic data

// Key functions:
CAN_ProcessRxMessage()      // Handle received
CAN_SendStatus()            // Transmit status
CAN_SendDiagnostics()       // Send fault info
```

#### stm32f4xx_it.c
**Purpose**: Interrupt handlers
```c
// What it does:
- SysTick interrupt (100µs timing)
- CAN receive interrupt
- ADC conversion complete
- Timer interrupts

// Key interrupts:
SysTick_Handler()           // Main timing
CAN1_RX0_IRQHandler()       // CAN receive
ADC_IRQHandler()            // ADC complete
TIM2_IRQHandler()           // Timer overflow
```

#### CheckTable.c
**Purpose**: Lookup tables and calibration
```c
// What it does:
- Stores calibration curves
- Linearization tables
- Compensation data

// Tables:
- Sensor linearization
- Temperature compensation
- Pressure curves
```

---

### Layer 3: STM32 Standard Peripheral Library (FWLIB/)

**Purpose**: Low-level hardware drivers provided by ST
- Pre-written, tested drivers
- One file per peripheral (GPIO, ADC, TIM, etc.)
- You typically don't modify these
- Called by your hardware abstraction layer

---

### Layer 4: System Core (CORE/)

#### startup_stm32f40_41xxx.s
**Purpose**: Startup code (assembly)
```
- Sets up stack pointer
- Initializes data sections
- Calls main()
- Defines interrupt vector table
```

#### system_stm32f4xx.c
**Purpose**: System initialization
```c
- Configures system clock (168MHz)
- Sets up flash wait states
- Initializes FPU
```

---

## 4. Key Concepts

### A. Control Loop Fundamentals

**PID Control Explained:**
```
Error = Setpoint - Feedback

P (Proportional): Immediate response to error
  Output_P = Kp × Error
  
I (Integral): Eliminates steady-state error
  Output_I = Ki × ∫Error dt
  
D (Derivative): Dampens oscillations
  Output_D = Kd × dError/dt

Total Output = Output_P + Output_I + Output_D
```

**Example from your code:**
```c
// Pressure control
pressure_error = target_pressure - actual_pressure;
pressure_output = PID_Calculate(&pressure_pid, pressure_error);
angle_command = pressure_output;  // Angle controls pressure
```

### B. Task Scheduling

**Cooperative Multitasking:**
```c
void TaskProcess(void)  // Called every 100µs
{
    static uint16_t counter_1ms = 0;
    static uint16_t counter_10ms = 0;
    
    // Fast tasks (100µs)
    ReadSensors();
    RunControlLoops();
    UpdateOutputs();
    
    // Medium tasks (1ms)
    if(++counter_1ms >= 10) {
        counter_1ms = 0;
        CAN_Process();
        Diagnostics();
    }
    
    // Slow tasks (10ms)
    if(++counter_10ms >= 100) {
        counter_10ms = 0;
        UpdateParameters();
    }
}
```

### C. Data Structures

**Global State Variables:**
```c
// Command/setpoint structure
struct InstructionSet {
    uint16_t TiltAngRef;          // Target angle (0-10000)
    uint16_t g_u16_PerUnitPrsVal; // Target pressure
    uint16_t Cur_A_Ref;           // Current A setpoint
    uint16_t Cur_B_Ref;           // Current B setpoint
};

// Feedback structure
struct DetectorFdbVal {
    uint32_t g_u32_AngFdb;        // Angle feedback
    int32_t g_s32_CurFdb_A;       // Current A feedback
    int32_t g_s32_CurFdb_B;       // Current B feedback
};

// Configuration structure
struct User_Parameter {
    PpqEna_t *PpqEna;             // Enable flags
    PID_Params *PID_Pressure;     // PID gains
    PID_Params *PID_Angle;
    // ... more parameters
};
```

### D. State Machine

**Pump Control States:**
```
IDLE → STARTUP → RUNNING → STOPPING → FAULT
  ↑                           ↓
  └───────────────────────────┘
```

```c
switch(pump_state) {
    case IDLE:
        // Wait for start command
        if(start_command) pump_state = STARTUP;
        break;
        
    case STARTUP:
        // Ramp up slowly
        if(pressure_reached) pump_state = RUNNING;
        break;
        
    case RUNNING:
        // Normal operation
        if(stop_command) pump_state = STOPPING;
        if(fault_detected) pump_state = FAULT;
        break;
        
    case STOPPING:
        // Ramp down
        if(pressure_zero) pump_state = IDLE;
        break;
        
    case FAULT:
        // Safe mode
        if(fault_cleared) pump_state = IDLE;
        break;
}
```

---

## 5. Code Reading Path

### For Beginners: Start Here

**Step 1: Understand main.c**
```
File: main.c
Focus on:
- main() function flow
- Initialization sequence
- Main loop structure
```

**Step 2: Follow the timing**
```
File: Task.c
Understand:
- How TaskProcess() is called
- Task timing (100µs, 1ms, 10ms)
- Task priorities
```

**Step 3: Trace sensor reading**
```
File: analog_input.c
Follow:
- How sensors are read
- ADC conversion
- Data filtering
```

**Step 4: Understand control**
```
File: application.c
Study:
- Control loop structure
- PID implementation
- State machine
```

### For Intermediate: Deep Dive

**Path 1: Pressure Control**
```
1. main.c → TaskProcess()
2. Task.c → calls application functions
3. analog_input.c → ReadPressureSensor()
4. application.c → PressureControlLoop()
5. algorithm.c → PID_Calculate()
6. limit.c → ApplyLimits()
7. Hw_TIM.c → UpdatePWM()
```

**Path 2: CAN Communication**
```
1. Hw_CAN.c → CAN initialization
2. stm32f4xx_it.c → CAN_RX interrupt
3. can_protocol.c → ProcessMessage()
4. application.c → Update setpoints
5. can_protocol.c → SendStatus()
```

**Path 3: Parameter Storage**
```
1. main.c → InitUserParaProcess()
2. flash.c → LoadFromFlash()
3. application.c → Use parameters
4. flash.c → SaveToFlash() (on command)
```

### For Advanced: System Design

**Study these interactions:**
1. Timing and synchronization
2. Interrupt priorities
3. Data consistency (volatile variables)
4. Safety mechanisms
5. Error recovery

---

## 6. Common Patterns

### Pattern 1: Hardware Initialization
```c
void Init_Peripheral(void)
{
    // 1. Enable clock
    RCC_APBxPeriphClockCmd(RCC_APBxPeriph_XXX, ENABLE);
    
    // 2. Configure GPIO (if needed)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_XXX;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
    
    // 3. Configure peripheral
    XXX_InitTypeDef XXX_InitStructure;
    XXX_InitStructure.XXX_Parameter = value;
    XXX_Init(XXXx, &XXX_InitStructure);
    
    // 4. Enable peripheral
    XXX_Cmd(XXXx, ENABLE);
    
    // 5. Enable interrupts (if needed)
    NVIC_EnableIRQ(XXX_IRQn);
}
```

### Pattern 2: Sensor Reading with Filtering
```c
uint16_t Read_Filtered_Sensor(void)
{
    static uint16_t filtered_value = 0;
    
    // 1. Read raw value
    uint16_t raw = ADC_GetValue();
    
    // 2. Apply low-pass filter
    // filtered = α × new + (1-α) × old
    filtered_value = (raw + filtered_value * 7) / 8;
    
    // 3. Apply calibration
    uint16_t calibrated = (filtered_value * SCALE) + OFFSET;
    
    // 4. Limit range
    if(calibrated > MAX) calibrated = MAX;
    if(calibrated < MIN) calibrated = MIN;
    
    return calibrated;
}
```

### Pattern 3: State Machine
```c
typedef enum {
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_ERROR
} State_t;

State_t current_state = STATE_IDLE;

void StateMachine_Process(void)
{
    switch(current_state) {
        case STATE_IDLE:
            // Entry actions
            if(first_time) {
                InitializeIdle();
                first_time = 0;
            }
            
            // State actions
            DoIdleStuff();
            
            // Transitions
            if(start_condition) {
                current_state = STATE_ACTIVE;
                first_time = 1;
            }
            break;
            
        case STATE_ACTIVE:
            // Similar structure
            break;
            
        case STATE_ERROR:
            // Similar structure
            break;
    }
}
```

### Pattern 4: CAN Message Handling
```c
void CAN_ProcessMessage(CanRxMsg *msg)
{
    switch(msg->StdId) {
        case MSG_ID_COMMAND:
            // Extract data
            uint16_t command = (msg->Data[0] << 8) | msg->Data[1];
            uint16_t value = (msg->Data[2] << 8) | msg->Data[3];
            
            // Process command
            ProcessCommand(command, value);
            break;
            
        case MSG_ID_PARAMETER:
            // Handle parameter update
            break;
            
        default:
            // Unknown message
            break;
    }
}
```

---

## Quick Reference: Where to Find Things

| What You Want | Where to Look |
|---------------|---------------|
| Main program flow | `main.c` |
| Control algorithms | `application.c` |
| PID implementation | `algorithm.c` |
| Sensor reading | `analog_input.c` |
| PWM output | `Hw_TIM.c` |
| CAN messages | `can_protocol.c` |
| Fault handling | `Diagnose.c` |
| Parameter storage | `flash.c` |
| Task timing | `Task.c` |
| Interrupt handlers | `stm32f4xx_it.c` |
| GPIO setup | `Hw_GPIO.c` |
| ADC setup | `Hw_ADC.c`, `AD7689.c` |

---

## Next Steps

1. **Read main.c** - Understand program flow
2. **Trace one sensor** - Follow from ADC to control output
3. **Study one control loop** - Understand PID implementation
4. **Examine CAN protocol** - See how commands work
5. **Review diagnostics** - Understand fault handling

**Pro Tip**: Use the debugger to watch variables in real-time. Set breakpoints in TaskProcess() and step through the code to see the execution flow.
