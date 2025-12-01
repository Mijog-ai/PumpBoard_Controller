# MCUViewer Setup Guide for PumpBoard Controller

## Quick Start

### 1. Hardware Connection

```
ST-Link V2/V3        STM32F4 Board
─────────────        ─────────────
SWDIO    ──────────  SWDIO
SWCLK    ──────────  SWCLK
GND      ──────────  GND
3.3V     ──────────  3.3V (optional)
```

### 2. Software Setup

1. **Download MCUViewer**
   ```bash
   # Linux/Mac
   wget https://github.com/klonyyy/MCUViewer/releases/latest/download/MCUViewer-Linux.AppImage
   chmod +x MCUViewer-Linux.AppImage
   ./MCUViewer-Linux.AppImage

   # Windows
   # Download MCUViewer-Windows.zip from releases page
   ```

2. **Install ST-Link Drivers** (if using ST-Link)
   - Windows: https://www.st.com/en/development-tools/stsw-link009.html
   - Linux: Install `stlink` package or use libusb

### 3. Launch MCUViewer

1. Start MCUViewer application
2. **Load ELF file**:
   - File → Load ELF
   - Select: `PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/OBJ/Template.axf`

3. **Connect debugger**:
   - Select probe type (ST-Link/J-Link)
   - Click "Connect"
   - MCU should be detected automatically

4. **Add variables to plot**:
   - Click "Add Variable"
   - Type variable name (e.g., `InstructionSet.TiltAngRef`)
   - Press Enter
   - Variable will appear in the plot

### 4. Essential Variables to Monitor

#### For PID Tuning - Monitor These Together:

**Angle Loop:**
```
InstructionSet.TiltAngRef          (red line - setpoint)
DetectorFdbVal.g_u32_AngFdb        (blue line - actual)
Angle_Loop.err                     (green line - error)
g_u32_PWMOutput_A                  (yellow - control output)
```

**Pressure Loop:**
```
InstructionSet.g_u16_PerUnitPrsVal (red line - setpoint)
DetectorFdbVal.g_u32_PressureFdb   (blue line - actual)
Pressure_Loop.err                  (green line - error)
```

**Current Loop:**
```
InstructionSet.Cur_B_Ref           (red line - setpoint)
DetectorFdbVal.g_s32_CurFdb_B      (blue line - actual)
Pwm_Output_B.err                   (green line - error)
```

### 5. Real-Time PID Parameter Tuning

While system is running, you can **modify PID gains**:

1. Right-click on variable (e.g., `Angle_Loop.Kp`)
2. Select "Modify Value"
3. Enter new value
4. Change takes effect immediately (within 100μs!)

**Example: Tune Angle Loop**
```
1. Monitor: TiltAngRef, g_u32_AngFdb, Angle_Loop.err
2. Adjust: Angle_Loop.Kp (increase if response too slow)
3. Observe: Watch error signal and settling time
4. Repeat: Fine-tune Kd for damping
```

## Advanced Features

### 6. Plot Configuration

- **Sampling Rate**: Set to 1ms (matches your control loops)
- **Plot Duration**: 5-10 seconds for transient response
- **Multiple Plots**: Create separate plots for each loop
- **Trigger**: Set trigger on setpoint change to capture step response

### 7. Data Export

- **Export to CSV**: File → Export Data
- **Analyze in Python/MATLAB**: Post-process for Bode plots, step response analysis

### 8. Memory Monitor

View live memory regions:
- Stack usage
- Heap usage
- Global variables
- Check for overflows

## Troubleshooting

### Connection Issues

**Problem**: "Could not connect to target"
```bash
# Linux: Check permissions
sudo usermod -a -G plugdev $USER
sudo udevadm control --reload-rules

# Check ST-Link firmware
st-info --probe
```

**Problem**: "ELF file doesn't match running code"
- Make sure you flashed the exact same .axf/.hex file
- Rebuild project and reflash

### Variable Not Found

**Problem**: "Variable 'xyz' not found in ELF"
- Check variable is global (extern)
- Ensure debug symbols enabled (-g flag)
- Variable might be optimized out - add `volatile` keyword

**Example fix:**
```c
// Before (might be optimized away)
extern u32 g_u32_PWMOutput_A;

// After (guaranteed visible)
extern volatile u32 g_u32_PWMOutput_A;
```

## Recommended Monitoring Setups

### Setup 1: Cascade Control Overview
Monitor all three loops simultaneously:
- Plot 1: Pressure (setpoint + feedback)
- Plot 2: Angle (setpoint + feedback)
- Plot 3: Current (setpoint + feedback)
- Plot 4: All PWM outputs

### Setup 2: PID Tuning Focus
Monitor single loop in detail:
- Setpoint
- Feedback
- Error
- P term (Kp × error)
- I term (integral)
- D term (Kd × derivative)
- Output

### Setup 3: System Health
- CPU usage (if available)
- Stack high water mark
- Control loop timing
- ADC values
- Sensor validity flags

## Example Session

```
1. Connect ST-Link to PumpBoard
2. Power on PumpBoard
3. Launch MCUViewer
4. Load Template.axf
5. Connect to ST-Link
6. Add variables:
   - InstructionSet.TiltAngRef
   - DetectorFdbVal.g_u32_AngFdb
   - Angle_Loop.Kp
7. Start acquisition
8. Change setpoint in your PC software or modify InstructionSet.TiltAngRef
9. Observe real-time response
10. Adjust Kp/Ki/Kd while running
11. Export data when satisfied
```

## Benefits of MCUViewer for Your System

✓ **Non-intrusive**: No code changes needed
✓ **Real-time**: 100μs update rate visible
✓ **Live tuning**: Change PID gains without recompiling
✓ **Multiple variables**: Monitor entire cascade at once
✓ **Data export**: Analyze step response, stability
✓ **Debug**: See exactly what's happening inside ISR

## Safety Notes

⚠️ **Important:**
- MCUViewer reads memory via debug interface
- Does NOT halt CPU (non-intrusive)
- System continues running normally
- Safe to use on running hydraulic system
- Can modify variables - be careful with safety-critical values!

## Next Steps

1. Try monitoring basic variables first
2. Gradually add more complex structures
3. Set up multi-plot layouts
4. Use triggers to capture transients
5. Export data for offline analysis
6. Fine-tune PID parameters in real-time

---

**Pro Tip**: Save your variable configurations in MCUViewer so you don't have to re-add them each session!
