# Hardware Test - Ready to Run! ✓

## Test Firmware Status: BUILT & READY

Your hardware test firmware has been successfully compiled and is ready to flash to your STM32F407ZE board.

---

## Quick Start (3 Steps)

### 1. Connect Hardware
```
- Connect ST-Link or J-Link debugger
- Power on the STM32F407ZE board
- Ensure LEDs are connected to PF9 and PF10 (or adjust pins in code)
```

### 2. Flash Test Firmware
```cmd
run_test.bat
```
Then select option 1 to flash.

### 3. Observe Results
**Current Test: GPIO LED Blink**
- LED on PF9 blinks: 100ms ON, 100ms OFF
- LED on PF10 blinks: 100ms ON, 100ms OFF  
- They alternate (when one is ON, the other is OFF)

---

## What's Currently Running

**Test Mode:** GPIO Test  
**What it does:** Blinks two LEDs alternately to verify GPIO functionality  
**Expected behavior:** Steady, alternating blink pattern  
**Success criteria:** Both LEDs blink at regular intervals  

---

## Available Tests

You can switch between different hardware tests:

### 1. GPIO Test (Currently Active)
- **Tests:** Digital output pins
- **Hardware needed:** 2 LEDs on PF9, PF10
- **Expected:** Alternating blink pattern
- **Duration:** Runs continuously

### 2. ADC Test
- **Tests:** Analog-to-digital converter
- **Hardware needed:** Variable voltage source (0-3.3V) on PA0
- **Expected:** LED on PF9 changes based on input voltage
- **Monitor:** `g_test_adc_value` in debugger (0-4095)

### 3. PWM Test
- **Tests:** Timer PWM output
- **Hardware needed:** Oscilloscope on PA1
- **Expected:** PWM signal sweeping 0-100% duty cycle
- **Frequency:** ~1kHz PWM

### 4. SPI Test
- **Tests:** SPI communication
- **Hardware needed:** Wire connecting PC12 (MOSI) to PC11 (MISO)
- **Expected:** Green LED (PF9) ON = success, Red LED (PF10) ON = fail

### 5. All Tests
- **Tests:** Sequential test pattern
- **Expected:** Combined test sequence

---

## How to Change Tests

### Option 1: Using run_test.bat (Easy)
```cmd
run_test.bat
```
Select option 2, choose your test, firmware rebuilds automatically.

### Option 2: Manual (Advanced)
1. Edit `main_test.c`
2. Uncomment the test you want:
```c
// #define TEST_MODE_GPIO      1
#define TEST_MODE_ADC       1    // <-- Uncomment this one
// #define TEST_MODE_PWM       1
```
3. Rebuild:
```cmd
copy main_test.c main.c
cbuild PumpCtrl.csolution.yml --rebuild
```

---

## Files Created

```
✓ main_test.c              - Test firmware source code
✓ main.c.backup            - Backup of original main.c
✓ run_test.bat             - Interactive test runner
✓ restore_main.bat         - Restore normal firmware
✓ test_hardware.c          - Test function library
✓ RUN_HARDWARE_TEST.md     - Detailed test guide
✓ DEBUG_GUIDE.md           - Complete debugging guide
✓ DEPLOYMENT_CHECKLIST.md  - Production deployment guide
```

---

## Test Results Monitoring

### Using Keil Debugger
1. Open `PumpCtrl.uvprojx`
2. Start debug (Ctrl+F5)
3. Watch these variables:
   - `g_test_counter` - Increments each test cycle
   - `g_test_adc_value` - ADC reading (ADC test)
   - `g_test_spi_result` - SPI received byte (SPI test)

### Using LED Indicators
- **GPIO Test:** Alternating blink = working
- **ADC Test:** LED brightness/state changes with input
- **PWM Test:** Status LED toggles = test running
- **SPI Test:** Green LED = pass, Red LED = fail

---

## After Testing

### All Tests Pass ✓
```cmd
restore_main.bat
```
This will:
1. Restore original main.c
2. Rebuild normal application firmware
3. Ready to flash production firmware

### Tests Fail ✗
1. Check hardware connections
2. Verify pin assignments match your board
3. Review `RUN_HARDWARE_TEST.md` for troubleshooting
4. Check `DEBUG_GUIDE.md` for detailed debugging steps

---

## Current Build Info

```
Firmware: Hardware Test Version
Test Mode: GPIO LED Blink
Target: STM32F407ZETx
Compiler: ARM Compiler 6 (AC6)
Binary: out/PumpCtrl/PPQ_Ctrl/PumpCtrl_TEST.bin
Size: Code=22KB, RO-data=218KB, RW-data=232B, ZI-data=1.9KB
Status: ✓ Built Successfully
```

---

## Safety Notes

⚠️ **Important:**
- Test firmware does NOT run the pump control application
- Watchdog is disabled during tests
- Tests run indefinitely - use debugger or power cycle to stop
- Safe to run without pump hardware connected
- Verify pin assignments before connecting hardware

---

## Next Steps

1. **Run the test:**
   ```cmd
   run_test.bat
   ```

2. **Verify hardware works:**
   - Check LED blinking pattern
   - Use oscilloscope for PWM/SPI tests
   - Monitor variables in debugger

3. **Try other tests:**
   - Use `run_test.bat` option 2 to switch tests
   - Test each hardware component individually

4. **When done:**
   ```cmd
   restore_main.bat
   ```

5. **Deploy production firmware:**
   - Follow `DEPLOYMENT_CHECKLIST.md`
   - Flash normal application
   - Run full system tests

---

## Support & Documentation

- **Quick test guide:** `RUN_HARDWARE_TEST.md`
- **Debugging help:** `DEBUG_GUIDE.md`
- **Deployment guide:** `DEPLOYMENT_CHECKLIST.md`
- **Test source code:** `main_test.c`

---

## Summary

✓ Test firmware built successfully  
✓ GPIO test ready to run  
✓ All test modes available  
✓ Easy switching between tests  
✓ Safe to run on hardware  
✓ Backup of original firmware saved  

**You're ready to test your hardware!**

Run `run_test.bat` to get started.
