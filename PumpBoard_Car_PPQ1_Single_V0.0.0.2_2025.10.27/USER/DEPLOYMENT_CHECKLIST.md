# STM32F407ZE PumpCtrl - Deployment Checklist

## Pre-Deployment Testing

### Phase 1: Hardware Verification
```
[ ] Power supply stable (3.3V ±5%)
[ ] All connections secure
[ ] No short circuits detected
[ ] Debugger connection working
[ ] LED indicators functional
```

### Phase 2: Component Testing
```
[ ] GPIO outputs toggle correctly
[ ] ADC readings within expected range
[ ] PWM signals verified with oscilloscope
[ ] SPI communication working (AD7689)
[ ] CAN bus communication established
[ ] Watchdog timer configured properly
```

### Phase 3: Sensor Calibration
```
[ ] Pressure sensor calibrated
[ ] Current sensors calibrated (A & B)
[ ] Angle sensor calibrated
[ ] Temperature sensors (if any) calibrated
[ ] All calibration values stored in flash
```

### Phase 4: Control Loop Testing
```
[ ] Pressure control loop stable
[ ] Angle control loop stable
[ ] Power control loop stable
[ ] PID parameters tuned
[ ] No oscillations or instability
[ ] Response time acceptable
```

### Phase 5: Safety Features
```
[ ] Over-current protection working
[ ] Over-pressure protection working
[ ] Emergency stop functional
[ ] Fault detection and recovery tested
[ ] Watchdog reset tested
[ ] CAN communication timeout handling
```

---

## Deployment Steps

### Step 1: Prepare Firmware
```cmd
# 1. Ensure simulation mode is OFF
# Check main.c line 137: #if 0

# 2. Set production parameters
# Edit configuration values in application.c

# 3. Build release version
cbuild PumpCtrl.csolution.yml

# 4. Verify binary size
# Check: out/PumpCtrl/PPQ_Ctrl/PumpCtrl.bin
```

### Step 2: Flash Firmware
```cmd
# Option A: Using flash.bat
flash.bat

# Option B: Manual ST-Link
STM32_Programmer_CLI -c port=SWD -w out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin 0x08000000 -v -rst

# Option C: Manual J-Link
JLink.exe -device STM32F407ZE -if SWD -speed 4000
# Then: loadbin out\PumpCtrl\PPQ_Ctrl\PumpCtrl.bin 0x08000000
```

### Step 3: Initial Power-On Test
```
[ ] Apply power
[ ] Check LED indicators
[ ] Verify no error codes
[ ] Monitor current consumption
[ ] Check for abnormal heating
[ ] Verify CAN communication
```

### Step 4: Functional Testing
```
[ ] Test all control modes
[ ] Verify sensor readings
[ ] Test manual control
[ ] Test automatic control
[ ] Verify data logging (if applicable)
[ ] Test emergency stop
```

### Step 5: Load Testing
```
[ ] Run at 25% load for 10 minutes
[ ] Run at 50% load for 10 minutes
[ ] Run at 75% load for 10 minutes
[ ] Run at 100% load for 10 minutes
[ ] Monitor temperature throughout
[ ] Check for any faults or warnings
```

### Step 6: Documentation
```
[ ] Record firmware version
[ ] Document calibration values
[ ] Note any hardware modifications
[ ] Record test results
[ ] Create deployment report
```

---

## Production Configuration

### Firmware Settings to Verify

```c
// In main.c
#define SIMULATION_MODE 0        // MUST be 0 for production

// Watchdog settings (adjust for your application)
IWDG_Init(4, 500);              // ~1 second timeout

// Control loop enables
User_Parameter.PpqEna->PumpStrt_Ena = 1;
User_Parameter.PpqEna->PrsLoop_Ena = 1;
User_Parameter.PpqEna->AngLeak_Ena = 1;
User_Parameter.PpqEna->PwrLoop_Ena = 1;
```

### Hardware Configuration

```
Jumpers/DIP Switches:
[ ] BOOT0 = LOW (normal operation)
[ ] BOOT1 = don't care
[ ] CAN termination = as required
[ ] Power selection = correct voltage

Connections:
[ ] All connectors secure
[ ] Cable strain relief adequate
[ ] No exposed conductors
[ ] Proper grounding
```

---

## Troubleshooting During Deployment

### Issue: Device Won't Start
**Check:**
1. BOOT0 pin state (should be LOW)
2. Power supply voltage
3. Reset circuit
4. Crystal oscillator (if external)

**Debug:**
```c
// Add to beginning of main()
GPIO_SetBits(GPIOF, GPIO_Pin_9);  // Turn on LED immediately
// If LED doesn't light, hardware issue
```

### Issue: Erratic Behavior
**Check:**
1. Power supply noise
2. Ground loops
3. EMI from motors/pumps
4. Loose connections

**Debug:**
```c
// Add watchdog feed counter
static uint32_t wdg_counter = 0;
void TaskProcess(void)
{
    wdg_counter++;
    IWDG_Feed();
    // Monitor wdg_counter to verify task is running
}
```

### Issue: CAN Communication Lost
**Check:**
1. CAN bus termination (120Ω at each end)
2. Cable length and quality
3. Baud rate match
4. Bus-off condition

**Debug:**
```c
// Check CAN error status
uint8_t tx_err = CAN_GetLSBTransmitErrorCounter(CAN1);
uint8_t rx_err = CAN_GetReceiveErrorCounter(CAN1);
// If > 127, bus-off condition
```

### Issue: Sensor Readings Incorrect
**Check:**
1. Sensor power supply
2. Reference voltage (VREF+)
3. ADC configuration
4. Calibration values

**Debug:**
```c
// Read raw ADC values
uint16_t raw_adc = ADC_GetConversionValue(ADC1);
// Should be 0-4095 for 12-bit ADC
// 0V = 0, 3.3V = 4095
```

---

## Post-Deployment Monitoring

### First 24 Hours
```
[ ] Check every 2 hours
[ ] Monitor temperature
[ ] Check for error codes
[ ] Verify data logs
[ ] Note any anomalies
```

### First Week
```
[ ] Daily inspection
[ ] Review error logs
[ ] Check wear on mechanical parts
[ ] Verify calibration still accurate
[ ] Monitor performance metrics
```

### Ongoing Maintenance
```
[ ] Weekly visual inspection
[ ] Monthly calibration check
[ ] Quarterly full system test
[ ] Annual firmware update review
```

---

## Firmware Update Procedure

### For Field Updates

1. **Prepare Update Package**
   ```
   - New firmware binary
   - Release notes
   - Update instructions
   - Rollback plan
   ```

2. **Backup Current Configuration**
   ```c
   // Read parameters from flash before update
   Save_Parameters_To_File();
   ```

3. **Flash New Firmware**
   ```cmd
   flash.bat
   # Or use ST-Link Utility for field updates
   ```

4. **Restore Configuration**
   ```c
   // Load saved parameters
   Load_Parameters_From_File();
   ```

5. **Verify Operation**
   ```
   [ ] All sensors reading correctly
   [ ] Control loops stable
   [ ] CAN communication working
   [ ] No error codes
   ```

---

## Emergency Procedures

### If System Becomes Unstable

1. **Immediate Actions**
   - Press emergency stop
   - Disconnect power if necessary
   - Note error codes/symptoms

2. **Safe Mode Boot**
   ```c
   // Add safe mode check in main()
   if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
   {
       // Enter safe mode - minimal functionality
       Safe_Mode_Operation();
   }
   ```

3. **Rollback to Previous Firmware**
   - Keep backup of last known good firmware
   - Flash using ST-Link Utility
   - Restore previous configuration

### Contact Information
```
Hardware Support: [Your contact]
Software Support: [Your contact]
Emergency: [Emergency contact]
```

---

## Version Control

### Firmware Version Tracking

```c
// Add to main.c
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  1
#define FW_VERSION_PATCH  0
#define FW_BUILD_DATE     __DATE__
#define FW_BUILD_TIME     __TIME__

// Store in flash for remote reading
const uint32_t firmware_version __attribute__((section(".version"))) = 
    (FW_VERSION_MAJOR << 16) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH;
```

### Deployment Log Template

```
Date: _______________
Firmware Version: _______________
Hardware Revision: _______________
Deployed By: _______________

Pre-Deployment Tests:
[ ] All tests passed
[ ] Issues noted: _______________

Deployment Notes:
_______________________________________________
_______________________________________________

Post-Deployment Verification:
[ ] System operational
[ ] Performance acceptable
[ ] No errors detected

Signature: _______________
```

---

## Success Criteria

System is ready for production when:
```
✓ All hardware tests pass
✓ All control loops stable
✓ Safety features verified
✓ 24-hour burn-in test completed
✓ Documentation complete
✓ Maintenance plan established
✓ Emergency procedures defined
✓ Support contacts identified
```

---

## Additional Resources

- Hardware schematic
- PCB layout
- Component datasheets
- Calibration procedures
- Maintenance manual
- Training materials

**Remember**: Safety first! Never deploy untested firmware to production systems.
