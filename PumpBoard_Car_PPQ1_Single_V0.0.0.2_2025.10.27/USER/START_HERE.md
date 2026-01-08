# 🚀 START HERE - Understanding Your PumpCtrl Code

## Welcome!

You have a complete pump control system for STM32F407ZE. This guide will help you understand how it all works.

---

## 📋 What You Have

### Your System Controls:
- **Hydraulic/Pneumatic Pump** with variable displacement
- **Sensors**: Pressure, current, angle, temperature
- **Control**: PID loops for pressure, angle, and power
- **Communication**: CAN bus for commands and status
- **Safety**: Watchdog, diagnostics, fault detection

### Project Size:
- **55 source files** organized in logical groups
- **~25KB code** + 220KB data
- **Real-time control** at 100µs (10kHz) update rate

---

## 🎯 Quick Start - Choose Your Path

### Path 1: I Want to Understand the Code (Recommended)
```
1. Read: CODE_UNDERSTANDING_GUIDE.md (30 min overview)
2. View: CODE_MAP.txt (visual diagrams)
3. Follow: LEARNING_PATH.md (step-by-step lessons)
4. Use: explore_code.bat (interactive explorer)
```

### Path 2: I Want to Test Hardware First
```
1. Read: HARDWARE_TEST_SUMMARY.md
2. Run: run_test.bat
3. Flash test firmware
4. Verify hardware works
5. Then return to Path 1
```

### Path 3: I Want to Debug/Modify
```
1. Read: DEBUG_GUIDE.md (comprehensive debugging)
2. Use: debug_session.bat (launch debugger)
3. Follow: DEPLOYMENT_CHECKLIST.md (for production)
```

---

## 📚 Documentation Overview

### Understanding the Code
| Document | Purpose | Time | Difficulty |
|----------|---------|------|------------|
| **CODE_UNDERSTANDING_GUIDE.md** | Complete code reference | 2-3 hours | ⭐⭐ |
| **CODE_MAP.txt** | Visual flow diagrams | 30 min | ⭐ |
| **LEARNING_PATH.md** | Step-by-step lessons | 10+ hours | ⭐⭐⭐ |
| **QUICK_REFERENCE.txt** | Command cheat sheet | 5 min | ⭐ |

### Testing & Debugging
| Document | Purpose | Time | Difficulty |
|----------|---------|------|------------|
| **HARDWARE_TEST_SUMMARY.md** | Test status & guide | 15 min | ⭐ |
| **RUN_HARDWARE_TEST.md** | Detailed test procedures | 1 hour | ⭐⭐ |
| **DEBUG_GUIDE.md** | Complete debugging guide | 2 hours | ⭐⭐⭐ |
| **DEPLOYMENT_CHECKLIST.md** | Production deployment | 1 hour | ⭐⭐⭐ |

### Build & Deploy
| Document | Purpose | Time | Difficulty |
|----------|---------|------|------------|
| **README_BUILD.md** | Build & flash guide | 15 min | ⭐ |

---

## 🛠️ Interactive Tools

### Scripts You Can Run

```cmd
explore_code.bat          # Interactive code explorer
                         # - View project structure
                         # - Open key files
                         # - Search for functions
                         # - View documentation

run_test.bat             # Hardware test runner
                         # - Flash test firmware
                         # - Switch between tests
                         # - Restore normal firmware

build.bat                # Build normal firmware
flash.bat                # Flash to hardware
restore_main.bat         # Restore from test mode
debug_session.bat        # Launch debugger
```

---

## 🗺️ Code Structure at a Glance

```
PumpCtrl/
│
├── USER/
│   └── main.c                    ⭐ START HERE - Entry point
│
├── Function/                     ⭐ CORE APPLICATION LOGIC
│   ├── application.c             → Control algorithms (PID loops)
│   ├── Task.c                    → Task scheduler (100µs timing)
│   ├── analog_input.c            → Sensor reading
│   ├── algorithm.c               → PID, filters, math
│   ├── can_protocol.c            → CAN communication
│   ├── flash.c                   → Parameter storage
│   ├── Diagnose.c                → Fault detection
│   └── limit.c                   → Safety limits
│
├── HardWare_Init/                ⭐ HARDWARE DRIVERS
│   ├── Hw_GPIO.c                 → GPIO setup
│   ├── Hw_ADC.c                  → ADC configuration
│   ├── Hw_TIM.c                  → PWM/Timer setup
│   ├── Hw_SPI.c                  → SPI communication
│   ├── Hw_CAN.c                  → CAN setup
│   └── AD7689.c                  → External ADC driver
│
├── FWLIB/                        → STM32 standard library
├── CORE/                         → System startup code
└── LIB/                          → Precompiled libraries
```

⭐ = Most important to understand

---

## 🎓 Learning Roadmap

### Week 1: Basics
```
Day 1-2: Read CODE_UNDERSTANDING_GUIDE.md
         Understand project overview and architecture

Day 3-4: Complete LEARNING_PATH.md Lessons 1-3
         Learn program flow and task scheduling

Day 5-7: Complete LEARNING_PATH.md Lessons 4-6
         Understand sensors, control, and outputs
```

### Week 2: Deep Dive
```
Day 1-3: Complete LEARNING_PATH.md Lessons 7-9
         Learn CAN, diagnostics, and parameters

Day 4-5: Complete LEARNING_PATH.md Lesson 10
         Trace complete data flow

Day 6-7: Work on graduation project
         Add a new feature
```

### Week 3: Mastery
```
Day 1-3: Debug and optimize
         Use DEBUG_GUIDE.md

Day 4-5: Test on hardware
         Follow HARDWARE_TEST_SUMMARY.md

Day 6-7: Deploy to production
         Follow DEPLOYMENT_CHECKLIST.md
```

---

## 🔑 Key Concepts to Understand

### 1. Real-Time Control
```
- System runs at 100µs (10kHz) update rate
- SysTick interrupt triggers every 100µs
- TaskProcess() called from main loop
- Fast tasks (100µs), medium (1ms), slow (10ms)
```

### 2. PID Control
```
- Pressure loop controls pump displacement
- Angle loop controls swash plate position
- Power loop limits maximum power
- Cascaded control: Pressure → Angle → Current → PWM
```

### 3. Data Flow
```
Sensors → ADC → Filter → Control → Limits → PWM → Actuators
   ↓                                                    ↑
   └──────────────► CAN Bus ◄────────────────────────┘
```

### 4. Safety
```
- Watchdog timer (must feed regularly)
- Sensor validation
- Limit checking
- Fault detection
- Safe mode on error
```

---

## 💡 Quick Tips

### Finding Things
```
Use explore_code.bat → Option 8 to search for:
- Function names
- Variable names
- Constants
- Comments
```

### Understanding Flow
```
1. Start at main.c
2. Follow to TaskProcess() in Task.c
3. Trace one sensor through the system
4. See how it affects control output
```

### Using Debugger
```
1. Set breakpoint in TaskProcess()
2. Watch these variables:
   - InstructionSet (commands)
   - DetectorFdbVal (sensor feedback)
   - g_u16_TestTim2_CCR2/4 (PWM outputs)
3. Step through code (F10)
4. Observe values change
```

### Modifying Code
```
1. Make small changes
2. Test one thing at a time
3. Use version control (git)
4. Document your changes
5. Test thoroughly before deployment
```

---

## 🆘 When You're Stuck

### Can't Find Something?
→ Use `explore_code.bat` → Option 8 (Search)

### Don't Understand Flow?
→ Read `CODE_MAP.txt` for visual diagrams

### Need Step-by-Step?
→ Follow `LEARNING_PATH.md` lessons

### Want to Debug?
→ Read `DEBUG_GUIDE.md`

### Hardware Not Working?
→ Follow `HARDWARE_TEST_SUMMARY.md`

---

## ✅ Your First Steps (Do This Now!)

### Step 1: Get Oriented (15 minutes)
```
1. Open CODE_MAP.txt
2. Read "EXECUTION FLOW" section
3. Understand: Power On → Init → Main Loop → TaskProcess
```

### Step 2: See the Code (15 minutes)
```
1. Run: explore_code.bat
2. Select Option 2 (Open main.c)
3. Find the main() function
4. Identify initialization and main loop
```

### Step 3: Choose Your Path (5 minutes)
```
Decide:
[ ] I want to learn systematically → LEARNING_PATH.md
[ ] I want quick reference → CODE_UNDERSTANDING_GUIDE.md
[ ] I want to test hardware → HARDWARE_TEST_SUMMARY.md
[ ] I want to debug/modify → DEBUG_GUIDE.md
```

---

## 📞 Support Resources

### Documentation
- All guides are in this folder
- Use explore_code.bat for quick access
- CODE_MAP.txt for visual reference

### Tools
- Keil µVision for debugging
- ST-Link or J-Link for flashing
- Oscilloscope for signal verification
- CAN analyzer for bus monitoring

### Community
- STM32 forums
- Embedded systems communities
- Control systems resources

---

## 🎯 Success Criteria

You'll know you understand the code when you can:

- [ ] Explain what happens from power-on to main loop
- [ ] Trace a sensor value from ADC to PWM output
- [ ] Describe how PID control works
- [ ] Modify a parameter and predict the result
- [ ] Add a new feature (like a sensor or output)
- [ ] Debug a problem using the debugger
- [ ] Deploy firmware to production safely

---

## 🚀 Ready to Start?

### Recommended First Action:
```cmd
1. Open CODE_MAP.txt (quick visual overview)
2. Run explore_code.bat → Option 1 (see structure)
3. Read CODE_UNDERSTANDING_GUIDE.md Section 1-2
4. Start LEARNING_PATH.md Lesson 1
```

### Or Jump Right In:
```cmd
explore_code.bat → Option 2 → Open main.c
```

---

## 📝 Notes

- Take your time - this is complex embedded code
- Use the debugger - seeing is understanding
- Make notes as you learn
- Test on hardware when possible
- Ask questions and research

**Remember**: Every expert was once a beginner. You've got this! 💪

---

**Good luck on your learning journey!** 🎉

For questions or issues, refer to the appropriate guide:
- Code questions → CODE_UNDERSTANDING_GUIDE.md
- Hardware issues → DEBUG_GUIDE.md
- Testing → HARDWARE_TEST_SUMMARY.md
- Learning → LEARNING_PATH.md
