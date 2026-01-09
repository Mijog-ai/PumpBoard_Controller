# Hardware Test Guide

## Quick Start

### Step 1: Build Test Firmware
```cmd
test_build.bat
```

Select the test you want to run:
1. **GPIO Test** - Blinks LEDs to verify GPIO functionality
2. **ADC Test** - Reads analog input and displays via LED
3. **PWM Test** - Generates PWM signal with sweeping duty cycle
4. **SPI Test** - Tests SPI communication (requires loopback)
5. **All Tests** - Runs all tests sequentially

### Step 2: Flash to Hardware
```cmd
flash.bat
```

### Step 3: Observe Results

#### GPIO Test
- **Expected**: LED on PF9 and PF10 blink alternately
- **Frequency**: 100ms on, 100ms off
- **Success**: Steady blinking pattern
- **Failure**: No LED activity or erratic blinking

#### ADC Test
- **Expected**: LED on PF9 changes based on ADC input
- **Setup**: Connect voltage source to PA0 (0-3.3V)
- **Success**: LED ON when voltage > 1.65V, OFF when < 1.65V
- **Monitor**: Watch `g_test_adc_value` in debugger (0-4095)

#### PWM Test
- **Expected**: PWM signal on PA1 (TIM2_CH2)
- **Measure**: Use oscilloscope on PA1
- **Success**: Duty cycle sweeps from 0% to 100% and back
- **Frequency**: ~1kHz PWM, 10ms sweep step
- **LED**: PF9 toggles to show test is running

#### SPI Test
- **Setup**: Connect PC12 (MOSI) to PC11 (MISO) for loopback
- **Expected**: 
  - Green LED (PF9) ON = SPI working
  - Red LED (PF10) ON = SPI failed
- **Success**: Green LED stays on
- **Monitor**: Watch `g_test_spi_result` in debugger

#### All Tests
- **Expected**: Sequential test pattern
- **Pattern**: 
  1. 10 fast blinks (GPIO)
  2. Both LEDs ON for 500ms
  3. Both LEDs OFF for 500ms
  4. Repeat

### Step 4: Restore Normal Firmware
```cmd
restore_main.bat
```

This will:
1. Restore original main.c
2. Rebuild normal firmware
3. Ready to flash normal application

---

## Hardware Connections

### GPIO Test
```
PF9  → LED1 (or oscilloscope)
PF10 → LED2 (or oscilloscope)
GND  → LED cathode
```

### ADC Test
```
PA0  → Analog input (0-3.3V)
PF9  → Status LED
GND  → Common ground
```

### PWM Test
```
PA1  → Oscilloscope probe (TIM2_CH2)
PF9  → Status LED
GND  → Common ground
```

### SPI Test
```
PC10 → SCK (can monitor with scope)
PC11 → MISO
PC12 → MOSI
Connect: PC12 to PC11 (loopback)
PF9  → Green LED (success)
PF10 → Red LED (fail)
```

---

## Debugging Test Results

### Using Keil Debugger

1. Build test firmware: `test_build.bat`
2. Open Keil: `PumpCtrl.uvprojx`
3. Start debug session: Ctrl+F5
4. Add watch variables:
   - `g_test_counter` - Increments each test cycle
   - `g_test_adc_value` - ADC reading (0-4095)
   - `g_test_spi_result` - SPI received byte
5. Run (F5) and monitor values

### Using Serial Monitor

Add UART output to tests:

```c
// In main_test.c, add to test loop:
printf("ADC: %d, Counter: %d\n", g_test_adc_value, g_test_counter);
```

Then monitor via serial terminal at 115200 baud.

---

## Troubleshooting

### No LED Activity
**Problem**: LEDs don't light up at all

**Check**:
1. Power supply (3.3V on VDD pins)
2. LED connections (correct polarity)
3. GPIO pin assignments match your board
4. Firmware flashed successfully

**Fix**:
```c
// Verify GPIO pins in main_test.c
// Change GPIO_Pin_9 to your actual LED pin
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  // Adjust this
```

### ADC Always Reads 0 or 4095
**Problem**: ADC stuck at min or max value

**Check**:
1. VREF+ connected to 3.3V
2. Analog input connected to PA0
3. Input voltage within 0-3.3V range
4. No short circuits

**Fix**:
```c
// Try different ADC channel
ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
// And change GPIO pin accordingly
```

### PWM Not Visible on Scope
**Problem**: No signal on oscilloscope

**Check**:
1. Correct pin (PA1 for TIM2_CH2)
2. Oscilloscope trigger settings
3. Voltage range (0-3.3V)
4. Timer clock enabled

**Fix**:
```c
// Verify timer is running
if(TIM2->CR1 & TIM_CR1_CEN)
{
    // Timer enabled - check pin configuration
}
```

### SPI Test Shows Red LED
**Problem**: SPI loopback failing

**Check**:
1. MOSI (PC12) physically connected to MISO (PC11)
2. Connection is solid (not loose)
3. No other devices on SPI bus
4. Correct pins for your board

**Fix**:
```c
// Add delay between send/receive
Delay_ms(1);
// Or check SPI clock polarity/phase
```

---

## Test Results Checklist

```
Hardware Test Results
Date: _______________
Board: STM32F407ZE
Firmware: Test Version

[ ] GPIO Test - PASS / FAIL
    Notes: _______________________________

[ ] ADC Test - PASS / FAIL
    ADC Value Range: _______ to _______
    Notes: _______________________________

[ ] PWM Test - PASS / FAIL
    Frequency: _______ Hz
    Duty Cycle Range: _______ %
    Notes: _______________________________

[ ] SPI Test - PASS / FAIL
    Loopback: SUCCESS / FAIL
    Notes: _______________________________

Overall Result: PASS / FAIL

Tested By: _______________
Signature: _______________
```

---

## Next Steps After Testing

### All Tests Pass ✓
1. Run `restore_main.bat`
2. Flash normal firmware
3. Proceed with application testing
4. Follow DEPLOYMENT_CHECKLIST.md

### Some Tests Fail ✗
1. Document which tests failed
2. Check hardware connections
3. Verify schematic matches code
4. Fix hardware issues
5. Re-run tests
6. Do NOT proceed to deployment until all tests pass

---

## Advanced: Custom Tests

### Add Your Own Test

1. Edit `main_test.c`
2. Add new test function:

```c
void Test_My_Hardware(void)
{
    // Your test code here
}
```

3. Add to main():

```c
#ifdef TEST_MODE_CUSTOM
    Test_My_Hardware();
#endif
```

4. Modify `test_build.bat` to add new option

---

## Safety Notes

⚠️ **Important**:
- Never apply voltage > 3.3V to GPIO pins
- Check polarity before connecting power
- Use current-limiting resistors with LEDs
- Disconnect motors/pumps during testing
- Keep watchdog disabled during tests
- Tests run indefinitely - use debugger to stop

---

## Support

If tests fail and you can't resolve:
1. Document exact failure symptoms
2. Note which test fails
3. Capture oscilloscope screenshots
4. Check DEBUG_GUIDE.md for more help
5. Review hardware schematic

**Remember**: All hardware tests must pass before deploying to production!
