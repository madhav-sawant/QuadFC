# QUADCOPTER PID TUNING - PROJECT STATUS
**Last Updated:** 2026-01-20

---

## 🚁 SYSTEM SPECS

- **Frame:** F450
- **Motors:** 1400KV (VERY POWERFUL - small errors = big movements)
- **Flight Controller:** ESP32 (new board - USB works fine now)
- **Firmware:** Custom ESP-IDF based
- **Control Loop:** Cascaded PID (Rate loop + Angle loop)

---

## 🎯 TUNING METHOD

Using **Betaflight "Inner-Loop-First" Systematic Method**:

1. ✅ **Phase 2: Rate P** - Find oscillation point, back off 30%
2. ✅ **Phase 3: Rate D** - Tune damping
3. ✅ **Phase 4: Rate I** - Tune rate holding
4. ⏳ **Phase 4.5: Yaw P** - Currently testing (yaw was too weak)
5. ⏳ **Phase 5: Angle P** - Later
6. ⏳ **Phase 6: Angle I** - Later

---

## 📊 TUNING PROGRESS - RESULTS FOUND

### **Rate Loop (Roll/Pitch) - DONE:**
```
Rate P: 0.036  ✅ (Best from Phase 2 testing)
Rate I: 0.015  ✅ (Best from Phase 4 testing)
Rate D: 0.020  ✅ (Best from Phase 3 testing)
```

### **Yaw - FLIGHT TESTING (Betaflight Standard Hybrid):**
**Status:** ZN tests showed instability. Switching to industry standard high-gain tuning for F450.
**Observation:** Low Roll/Pitch P caused Angle Loop Failure (drift to 58°).
**Fix:** Increased Roll/Pitch P to 0.25 to prevent crashing.

```
Current Configuration (Flight Test 1 & 2):
Yaw P: 0.290  ✅ (User confirmed feels proper)
Yaw I: 0.300  ✅ (High I to fight left drift bias)
Yaw D: 0.000  ✅ (Removed to prevent noise)

Roll/Pitch P: 0.250 (Increased from 0.036 to fix angle failure)
Roll/Pitch I: 0.100
Roll/Pitch D: 0.020
```

### **Angle Loop - NOT YET TUNED:**
```
Angle P: 0.20 (Default)
Angle I: 0.00 (Disabled for Rate tuning)
```

---

## 📁 BLACKBOX DATA LOCATIONS

All test data saved at:
```
/home/maddy/new/Firmware/blackbox/flight/      ← Rate P tests
/home/maddy/new/Firmware/blackbox/Flight D/    ← Rate D tests
/home/maddy/new/Firmware/blackbox/Flight I/    ← Rate I tests
/home/maddy/new/Firmware/blackbox/Yaw (2)/     ← ZN Yaw tests
```

---

## ⚙️ CURRENT CONFIG FILE VALUES

Location: `/home/maddy/new/Firmware/lib/config/config.c`

**Currently Flashed:**
```c
// ROLL PID - SAFETY BOOST
sys_cfg.roll_kp = 0.250f;
sys_cfg.roll_ki = 0.100f;
sys_cfg.roll_kd = 0.020f;

// PITCH PID - SAFETY BOOST
sys_cfg.pitch_kp = 0.250f;
sys_cfg.pitch_ki = 0.100f;
sys_cfg.pitch_kd = 0.020f;

// YAW PID - BETAFLIGHT STANDARD HYBRID
sys_cfg.yaw_kp = 0.290f;
sys_cfg.yaw_ki = 0.300f;
sys_cfg.yaw_kd = 0.000f;
```

---

## 🚀 NEXT STEPS

1. **Perform 2 Test Flights** with new values.
2. **Check Yaw Drift:** Does it hold heading? (I=0.300 should fix left drift)
3. **Check Roll/Pitch:** Does it stay level? (P=0.250 should fix 58° angle fail)
4. **Final Adjustments:** Tweaking I-term if minor drift remains.

---

## 📊 KEY FINDINGS FROM TESTING

1. **Rate P oscillation point:** ~0.043 (so optimal is 0.036) -> *Superseded by finding P=0.036 was too weak for angle holding*
2. **Yaw Bias:** Strong LEFT drift confirmed. Needs high I-term.
3. **Angle Failure:** Roll/Pitch P < 0.10 causes loss of control during yaw spins.
4. **Yaw Stability:** P=0.290 confirmed by user as "proper".

---

## 🔧 USEFUL COMMANDS

**Build firmware:**
```bash
cd /home/maddy/new/Firmware
pio run
```

**Upload to ESP32:**
```bash
cd /home/maddy/new/Firmware
pio run --target upload
```

**Monitor serial:**
```bash
pio device monitor
```

---

**21+ days of tuning - Finally making systematic progress!** 🚁
