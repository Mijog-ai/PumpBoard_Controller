# Quick Start Guide - STM32 PumpBoard GUI

## 5-Minute Setup

### Step 1: Install Python Dependencies

#### Windows
1. Double-click `run_gui.bat`
2. Dependencies will install automatically

#### Linux/Mac
```bash
chmod +x run_gui.sh
./run_gui.sh
```

Or manually:
```bash
pip install -r requirements.txt
python gui_main.py
```

### Step 2: Connect Hardware

1. Connect STM32F407ZE controller to PC via USB Type-B cable
2. Wait for driver installation (if needed)
3. Note the COM port:
   - **Windows**: Check Device Manager → Ports (COM3, COM4, etc.)
   - **Linux**: Usually `/dev/ttyUSB0` or `/dev/ttyACM0`
   - **Mac**: Usually `/dev/cu.usbserial-*`

### Step 3: Connect via GUI

1. Launch GUI (see Step 1)
2. Select your COM port from dropdown
3. Verify baudrate is **115200**
4. Click **Connect** button
5. Button turns green when connected ✓

### Step 4: Monitor Parameters

1. Go to **Dashboard** tab
2. Watch real-time values update:
   - Angle Feedback (%)
   - Pressure Feedback (bar)
   - Current A & B (mA)
   - PWM Outputs (%)
3. View live plots below

### Step 5: Control the System

1. Go to **Control** tab
2. Enable control loops:
   - ☑ Enable Pump Start
   - ☑ Enable Pressure Loop (if needed)
3. Set desired values:
   - Angle Reference (0-100%)
   - Pressure Reference (0-600 bar)
4. Click **Apply** for each parameter

## Safety First!

### Before Starting
- ✓ All enable flags should be OFF
- ✓ All setpoints should be ZERO
- ✓ Emergency stop button visible

### During Operation
- ⚠ Monitor pressure (max 600 bar)
- ⚠ Monitor current (max 3000 mA)
- ⚠ Watch for abnormal behavior
- ⚠ Use emergency stop if needed

### Emergency Stop
Click the red **EMERGENCY STOP** button to:
- Disable all control loops
- Zero all setpoints
- Safe shutdown

## Common Operations

### Start Pump Control
1. Connect to controller
2. Read all parameters (Control tab → "Read All Parameters")
3. Enable Pump Start
4. Set small angle reference (e.g., 10%)
5. Click Apply
6. Gradually increase as needed

### Adjust Pressure
1. Enable Pressure Loop
2. Set pressure reference (e.g., 100 bar)
3. Click Apply
4. Monitor feedback in Dashboard

### Tune PID (Advanced)
1. Go to **PID Tuning** tab
2. Click "Read" to get current values
3. Adjust one parameter at a time
4. Click "Write" to apply
5. Test in Dashboard tab
6. Small changes only!

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't connect | Check COM port, USB cable, drivers |
| Timeout errors | Check baudrate (115200), try different cable |
| Values not updating | Verify connection, check firmware running |
| Can't write parameters | Ensure connected, check parameter range |

## Next Steps

- Read full **README.md** for detailed documentation
- Explore all tabs: Dashboard, Control, PID Tuning, Calibration
- Experiment with different setpoints (safely!)
- Monitor performance with real-time plots

## Need Help?

- Check **README.md** for detailed documentation
- Review protocol details and parameter mapping
- Verify firmware version matches GUI expectations

---

**Quick Tip**: Use "Read All Parameters" button to sync GUI with controller state!
