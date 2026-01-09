# STM32F407ZE PumpCtrl - Debugging & Development Guide

## Table of Contents
1. [Hardware Setup](#hardware-setup)
2. [Debugging Methods](#debugging-methods)
3. [Understanding the Code](#understanding-the-code)
4. [How to Modify Functionality](#how-to-modify-functionality)
5. [Common Issues & Solutions](#common-issues--solutions)

---

## 1. Hardware Setup

### Required Hardware
- **STM32F407ZE Development Board**
- **Debugger**: ST-Link V2/V3 or J-Link
- **Power Supply**: USB or external 5V
- **Pump Hardware**: Connected via GPIO, ADC, PWM, and CAN

### Pin Connections (Check your hardware schematic)
Based on the code, the system uses:
- **PWM Output**: TIM2 (for pump control)
- **ADC Input**: ADC1 + AD7689 (external ADC via SPI3)
- **CAN Bus**: For communication
- **SPI3**: For AD7689 external ADC
- **GPIO**: Various control signals

### Initial Hardware Checklist
```
[ ] Power supply connected (3.3V/5V)
[ ] Debugger connected (SWDIO, SWCLK, GND, 3.3V)
[ ] CAN transceiver connected (if using CAN)
[ ] Pump hardware connected
[ ] Sensors connected (pressure, current, angle)
```

---

## 2. Debugging Methods

### Method 1: Keil µVision Debugger (Recommended)

#### Setup Steps:
1. Open `PumpCtrl.uvprojx` in Keil µVision
2. Configure debugger:
   - Project → Options for Target
   - Debug tab → Select "ST-Link Debugger" or "J-Link"
   - Settings → Check connection
3. Build project (F7)
4. Start debug session (Ctrl+F5)

#### Debug Features:
- **Breakpoints**: Click on line numbers
- **Watch Variables**: Add global variables to watch window
- **Memory View**: View/edit memory in real-time
- **Peripheral View**: Monitor STM32 peripherals (GPIO, ADC, TIM, etc.)
- **Logic Analyzer**: Trace signal timing

#### Key Variables to Watch:
```c
// Control enables
User_Parameter.PpqEna->PumpStrt_Ena
User_Parameter.PpqEna->PrsLoop_Ena
User_Parameter.PpqEna->AngLeak_Ena
User_Parameter.PpqEna->PwrLoop_Ena

// Setpoints
InstructionSet.TiltAngRef        // Angle reference (0-10000)
InstructionSet.g_u16_PerUnitPrsVal  // Pressure reference
InstructionSet.Cur_A_Ref         // Current A reference
InstructionSet.Cur_B_Ref         // Current B reference

// Feedback values
DetectorFdbVal.g_u32_AngFdb      // Angle feedback
DetectorFdbVal.g_s32_CurFdb_A    // Current A feedback
DetectorFdbVal.g_s32_CurFdb_B    // Current B feedback

// PWM outputs
g_u16_TestTim2_CCR2              // PWM channel 2
g_u16_TestTim2_CCR4              // PWM channel 4

// Simulation mode
g_u8_SimulationMode              // 0=real hardware, 1=simulation
```

### Method 2: Serial Debug Output (UART)

Add printf debugging via UART:

```c
// Add to your code
#include <stdio.h>

// Retarget printf to UART (add this function)
int fputc(int ch, FILE *f)
{
    // Wait until transmit data register is empty
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = (ch & 0xFF);
    return ch;
}

// Then use printf in your code
printf("Angle: %d, Current: %d\n", DetectorFdbVal.g_u32_AngFdb, DetectorFdbVal.g_s32_CurFdb_A);
```

### Method 3: LED Indicators

Add visual debugging:

```c
// Toggle LED to indicate program state
void DebugLED_Toggle(void)
{
    GPIO_ToggleBits(GPIOF, GPIO_Pin_9);  // Adjust pin as needed
}

// Call in main loop or specific functions
void TaskProcess(void)
{
    DebugLED_Toggle();  // Blink to show task is running
    // ... rest of code
}
```

### Method 4: Logic Analyzer / Oscilloscope

Monitor signals in real-time:
- **PWM signals**: TIM2 outputs
- **SPI communication**: MOSI, MISO, SCK, CS
- **CAN bus**: CAN_TX, CAN_RX
- **GPIO states**: Control signals

---

## 3. Understanding the Code

### Application Architecture

```
main()
  ├── Hardware Initialization
  │   ├── GPIO_Init_Func()           // GPIO setup
  │   ├── SPI3_Int()                 // SPI for external ADC
  │   ├── TIM2_init()                // PWM timer
  │   ├── Adc1_Analog_Input_Init()   // Internal ADC
  │   ├── AD7689InitFunc()           // External ADC
  │   └── CAN initialization
  │
  ├── Application Setup
  │   ├── InitUserParaProcess()      // Load parameters
  │   ├── SensorModeChangeFunc()     // Sensor configuration
  │   └── TaskParmInit()             // Task initialization
  │
  └── Main Loop
      └── TaskProcess()              // Periodic task execution
```

### Key Modules

#### 1. **Task System** (`Task.c`)
- Periodic task scheduler
- Runs control loops at fixed intervals
- Handles timing and synchronization

#### 2. **Application Logic** (`application.c`)
- Main control algorithms
- PID loops for pressure, angle, power
- State machine logic

#### 3. **Hardware Abstraction** (`HardWare_Init/`)
- `Hw_GPIO.c`: GPIO configuration
- `Hw_ADC.c`: ADC setup and reading
- `Hw_TIM.c`: Timer/PWM configuration
- `Hw_SPI.c`: SPI communication
- `Hw_CAN.c`: CAN bus interface
- `AD7689.c`: External ADC driver

#### 4. **Control Functions** (`Function/`)
- `algorithm.c`: Control algorithms
- `analog_input.c`: Sensor reading
- `flash.c`: Parameter storage
- `can_protocol.c`: CAN communication
- `Diagnose.c`: Fault detection

### System Timing

```c
SysTick: 100µs (10kHz)           // Main timing base
├── Task execution
├── ADC sampling
└── Control loop updates

Watchdog: ~1-2 seconds           // Safety timeout
└── Must call IWDG_Feed() regularly
```

---

## 4. How to Modify Functionality

### A. Change PWM Frequency/Duty Cycle

**Location**: `main.c` and `Hw_TIM.c`

```c
// In main.c
#define PWM_ARR 20000      // Change this for frequency
#define TIM2_PSC 84        // Prescaler

// Duty cycle control
g_u16_TestTim2_CCR2 = 5000;   // 25% duty (5000/20000)
g_u16_TestTim2_CCR4 = 10000;  // 50% duty (10000/20000)
```

**Steps**:
1. Modify `PWM_ARR` value in header or main.c
2. Rebuild project
3. Test with oscilloscope
4. Adjust duty cycle values as needed

### B. Add New Sensor Input

**Example**: Adding a temperature sensor on ADC channel

```c
// 1. In Hw_ADC.c - Add new ADC channel
void Adc1_Analog_Input_Init(void)
{
    // ... existing code ...
    
    // Add new channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_84Cycles);
}

// 2. Create reading function
u16 Read_Temperature_ADC(void)
{
    // Start conversion
    ADC_SoftwareStartConv(ADC1);
    
    // Wait for conversion
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // Read value
    return ADC_GetConversionValue(ADC1);
}

// 3. Add to task processing
void TaskProcess(void)
{
    static u16 temperature = 0;
    
    temperature = Read_Temperature_ADC();
    
    // Convert to actual temperature
    float temp_celsius = (temperature * 3.3f / 4096.0f - 0.5f) * 100.0f;
    
    // Use temperature value...
}
```

### C. Modify Control Loop Parameters

**Location**: `application.c` or parameter structures

```c
// PID tuning example
typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
} PID_Params;

// Adjust these values for your system
PID_Params pressure_pid = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.05f
};

// Test incrementally:
// 1. Start with Kp only
// 2. Add Ki for steady-state error
// 3. Add Kd for damping
```

### D. Add CAN Communication

**Location**: `can_protocol.c`

```c
// Send custom CAN message
void Send_Custom_CAN_Message(u32 id, u8* data, u8 length)
{
    CanTxMsg TxMessage;
    
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = length;
    
    for(u8 i = 0; i < length; i++)
    {
        TxMessage.Data[i] = data[i];
    }
    
    CAN_Transmit(CAN1, &TxMessage);
}

// Receive handler
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0))
    {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        // Process received message
        switch(RxMessage.StdId)
        {
            case 0x100:
                // Handle message
                break;
        }
        
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
```

### E. Store Parameters in Flash

**Location**: `flash.c`

```c
// Save parameters to flash
void Save_Parameters_To_Flash(void)
{
    FLASH_Unlock();
    
    // Erase sector
    FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    
    // Write data
    u32 address = 0x080E0000;  // Sector 11 start
    FLASH_ProgramWord(address, parameter_value);
    
    FLASH_Lock();
}

// Load parameters from flash
void Load_Parameters_From_Flash(void)
{
    u32 address = 0x080E0000;
    parameter_value = *(__IO u32*)address;
}
```

---

## 5. Common Issues & Solutions

### Issue 1: Program Doesn't Start
**Symptoms**: No LED activity, debugger can't connect

**Solutions**:
```
1. Check power supply (3.3V on VDD pins)
2. Check BOOT0 pin (should be LOW for normal operation)
3. Verify debugger connection (SWDIO, SWCLK, GND)
4. Try mass erase: ST-Link Utility → Target → Erase Chip
5. Check for short circuits on power rails
```

### Issue 2: Watchdog Reset Loop
**Symptoms**: Program restarts every few seconds

**Solutions**:
```c
// Option 1: Disable watchdog during development
// Comment out in main():
// IWDG_Init(4, 500);

// Option 2: Feed watchdog in main loop
void TaskProcess(void)
{
    IWDG_Feed();  // Add this
    // ... rest of code
}
```

### Issue 3: ADC Readings Incorrect
**Symptoms**: Sensor values are 0, 4095, or random

**Solutions**:
```
1. Check reference voltage (VREF+ = 3.3V)
2. Verify sensor connections
3. Check ADC channel configuration
4. Add filtering:
```

```c
// Software filter
u16 Filter_ADC_Reading(u16 new_value)
{
    static u16 filtered = 0;
    filtered = (filtered * 7 + new_value) / 8;  // Simple IIR filter
    return filtered;
}
```

### Issue 4: PWM Not Working
**Symptoms**: No output on PWM pins

**Solutions**:
```
1. Check GPIO alternate function configuration
2. Verify timer clock is enabled
3. Check PWM mode and polarity
4. Measure with oscilloscope, not multimeter
```

```c
// Debug PWM
void Debug_PWM_Status(void)
{
    u16 arr = TIM2->ARR;
    u16 ccr2 = TIM2->CCR2;
    u16 cnt = TIM2->CNT;
    
    // Check if timer is running
    if(TIM2->CR1 & TIM_CR1_CEN)
    {
        // Timer enabled
        // Duty cycle = CCR2/ARR
    }
}
```

### Issue 5: CAN Communication Fails
**Symptoms**: No CAN messages sent/received

**Solutions**:
```
1. Check CAN transceiver power
2. Verify CAN_H and CAN_L connections
3. Check termination resistors (120Ω at each end)
4. Verify baud rate matches other devices
5. Check for bus-off state
```

```c
// Check CAN status
void Debug_CAN_Status(void)
{
    u8 error_code = CAN_GetLastErrorCode(CAN1);
    u8 tx_error = CAN_GetLSBTransmitErrorCounter(CAN1);
    u8 rx_error = CAN_GetReceiveErrorCounter(CAN1);
    
    // If errors > 127, bus-off condition
    if(tx_error > 127 || rx_error > 127)
    {
        // Reset CAN
        CAN_DeInit(CAN1);
        // Re-initialize...
    }
}
```

---

## Quick Reference Commands

### Build & Flash
```cmd
build.bat              # Build project
flash.bat              # Flash to hardware
cbuild --clean         # Clean build
```

### Debug in Keil
```
F7                     # Build
Ctrl+F5                # Start debug
F5                     # Run
F10                    # Step over
F11                    # Step into
Shift+F5               # Stop debug
```

### Useful Registers to Monitor
```
RCC->CR                # Clock status
GPIOA->ODR             # GPIO output
ADC1->DR               # ADC data
TIM2->CNT              # Timer counter
CAN1->ESR              # CAN error status
```

---

## Next Steps

1. **Test Basic Functionality**
   - Flash firmware
   - Verify power-on LED
   - Check PWM outputs with scope
   - Test ADC readings

2. **Calibrate Sensors**
   - Record ADC values at known points
   - Create calibration curves
   - Store calibration in flash

3. **Tune Control Loops**
   - Start with low gains
   - Increase gradually
   - Monitor stability

4. **Add Safety Features**
   - Over-current protection
   - Temperature monitoring
   - Fault detection and recovery

5. **Optimize Performance**
   - Profile execution time
   - Optimize critical loops
   - Reduce interrupt latency

---

## Additional Resources

- **STM32F407 Reference Manual**: Detailed peripheral documentation
- **Keil µVision User Guide**: Debugger features
- **CAN Bus Specification**: Protocol details
- **Your Hardware Schematic**: Pin assignments and connections

For specific questions about your hardware setup, consult your board documentation.
