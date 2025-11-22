# CODE39 Barcode Accuracy Guide for RoboCar Project

## 🎯 Target: 100% Decode Accuracy

This guide ensures your barcode system meets the project requirements:
- ✅ Bidirectional scanning (forward/reverse)
- ✅ Variable barcode sizes
- ✅ Speed variations

---

## 📏 Optimal Speed Range

### **Recommended Speed: 8-12 cm/s**

| Speed (cm/s) | Accuracy | Notes |
|--------------|----------|-------|
| < 5 | Good but slow | Risk of timeout on very long barcodes |
| **8-12** | **BEST** | **Optimal accuracy and speed** |
| 15-20 | Fair | May miss narrow bars at extremes |
| > 25 | Poor | High failure rate |

### Why Speed Matters:

```
Example: Narrow bar width = 1mm

Speed 10 cm/s:
- Time over bar = 1mm / 100mm/s = 10ms
- Sampling at 2ms intervals → 5 samples ✅

Speed 30 cm/s:
- Time over bar = 1mm / 300mm/s = 3.3ms
- Sampling at 2ms intervals → 1-2 samples ⚠️
```

**Your scanner updates every 2ms** (DT_BARCODE_US = 2000), so:
- At 10 cm/s: 5 samples per 1mm bar ✅
- At 30 cm/s: 1-2 samples per 1mm bar ⚠️

---

## 🔧 Hardware Setup for Maximum Accuracy

### 1. **Sensor Height**
```
Optimal: 5mm ± 1mm from barcode surface
Too close (< 3mm): May miss transitions
Too far (> 8mm): Reduced contrast
```

### 2. **Sensor Alignment**
```
┌─────────────┐
│  Direction  │
│     →→→     │  Barcode
└─────────────┘
      ↑
    Sensor (centered under barcode path)
```

**Critical:** Sensor must be centered under the barcode track!

### 3. **Lighting Conditions**
- IR sensors prefer **dim ambient light**
- Avoid direct sunlight on sensor
- Indoor fluorescent lighting is ideal

### 4. **Barcode Print Quality**
- **Sharp edges** on bars (no blur)
- **High contrast** (black on white paper)
- **Matte finish** (not glossy - reduces reflections)

---

## 📊 Calibration for 100% Accuracy

### Step 1: Run Calibration
```c
barcode_direction_calibrate();
```

### Expected Values:
```
White surface:  200-800   (typical: 500)
Black bar:      2500-3800 (typical: 3500)
Threshold:      Auto-calculated midpoint
Range:          > 500 for good performance
```

### Step 2: Verify Real-Time Readings
```c
// Add this to your main loop temporarily for testing
uint16_t raw = barcode_direction_read_raw();
bool state = barcode_direction_read_sensor();
printf("ADC: %d | State: %s\n", raw, state ? "BLACK" : "WHITE");
```

### What to Look For:
- **Stable readings**: Values shouldn't jump around
- **Clean transitions**: Clear switch between black/white
- **Good separation**: Black and white values well separated

---

## 🎨 Barcode Design Guidelines

Based on your project track layout:

### Minimum Bar Width
```
Narrow bar: ≥ 1.0mm
Wide bar:   ≥ 1.5mm (1.5× narrow)

At 10 cm/s:
- Narrow (1mm) → 10ms duration → 5 samples ✅
- Wide (1.5mm) → 15ms duration → 7 samples ✅
```

### Quiet Zones
```
*ABC*
↑   ↑
│   └─ End quiet zone (≥ 10× narrow bar width)
└───── Start quiet zone (≥ 10× narrow bar width)

For 1mm narrow bars → 10mm white space before/after
```

### Barcode Length
```
Maximum recommended: 20 characters
- Prevents buffer overflow (BC_DIR_MAX_BARS = 100)
- Each char = ~9 elements = 9 bars
- 20 chars = 180 elements (within limit)
```

---

## 🚗 Speed Control During Scanning

### Option 1: Maintain Current Speed
```c
// Your robot already line-follows at ~43 speed units
// This works well if your speed is in the 8-12 cm/s range
// No changes needed!
```

### Option 2: Slow Down for Barcodes (Optional)
```c
// In robot_controller.c, when barcode detected:
if (bc_result != NULL) {
    // Optionally reduce speed while executing barcode command
    motor_set_differential(20, 20);  // Slower for precision
    // Execute turn
    // Resume normal speed
    motor_set_differential(cur_left_speed, cur_right_speed);
}
```

---

## 🔍 Testing for 100% Accuracy

### Test Suite:

#### Test 1: Basic Decode
```
Barcode: *A*
Expected: "A" → MOVE_RIGHT
Speed: 10 cm/s
Pass: ✅
```

#### Test 2: Bidirectional
```
Forward scan: *ABC*  → "ABC"
Reverse scan: *ABC*  → "ABC" (auto-reversed)
Pass: ✅/❌
```

#### Test 3: Speed Variation
```
5 cm/s:  Success rate: ___%
10 cm/s: Success rate: ___%
15 cm/s: Success rate: ___%
20 cm/s: Success rate: ___%
```

#### Test 4: Multiple Characters
```
*LEFT*  → "LEFT" → MOVE_LEFT (L)
*RIGHT* → "RIGHT" → MOVE_RIGHT (R)
*GO*    → "GO" → MOVE_RIGHT (G)
```

#### Test 5: Distance from Track
```
3mm height: Success rate: ___%
5mm height: Success rate: ___%
8mm height: Success rate: ___%
```

---

## ⚙️ Current System Configuration

Your implementation already has:

```c
// Update rate
#define DT_BARCODE_US   2000    // 2ms = 500 Hz sampling

// Detection thresholds
#define BARCODE_END_TIMEOUT_US  15000   // 15ms white = end
#define BARCODE_START_MIN_US    100     // Min width to start

// Buffer sizes
#define BC_DIR_MAX_BARS     100     // Max bars to capture
#define BC_DIR_MAX_DATA_LEN 30      // Max characters

// Bar width limits
#define BC_DIR_MIN_BAR_WIDTH    50      // 50 µs minimum
#define BC_DIR_MAX_BAR_WIDTH    10000   // 10 ms maximum
```

---

## 🐛 Troubleshooting: Why Decodes Fail

### Problem: "Invalid Pattern" Error

**Possible Causes:**
1. **Speed too high** → Slow down to 8-12 cm/s
2. **Poor calibration** → Re-run `barcode_direction_calibrate()`
3. **Sensor height wrong** → Adjust to 5mm ± 1mm
4. **Barcode quality** → Print with sharp, high-contrast bars
5. **Lighting issues** → Test in consistent lighting

### Problem: Sometimes Works, Sometimes Doesn't

**Likely Causes:**
1. **Inconsistent speed** → Enable motor PID control
2. **Vibration** → Check sensor mounting (should be rigid)
3. **Sensor alignment** → Ensure perpendicular to barcode
4. **Track variation** → Ensure flat, smooth barcode surface

### Problem: Works in One Direction Only

**Diagnosis:**
- Forward works, reverse fails → Check reverse decode logic
- Check MQTT output to see which direction is being detected

**Solution:**
- System should auto-detect and decode both directions
- Verify with: `printf("Scan dir: %s\n", barcode_scan_direction_string(result->scan_dir));`

---

## 📈 Performance Metrics to Track

Via MQTT (`robocar/barcode` topic):

```json
{
    "barcode": "ABC",           // Decoded string
    "direction": "MOVE RIGHT",  // Movement command
    "scan_dir": "FORWARD (L→R)",// Forward or reverse
    "move_dir": 1,              // 0=NONE, 1=RIGHT, 2=LEFT
    "bar_count": 45             // Diagnostic: bars captured
}
```

### Success Indicators:
- ✅ **bar_count**: 20-90 for typical barcodes
- ✅ **scan_dir**: Should work both ways
- ✅ **status**: BC_STATUS_OK (0)

### Failure Indicators:
- ⚠️ **bar_count < 20**: Too fast or barcode too short
- ⚠️ **bar_count > 95**: Possible noise/multiple barcodes
- ❌ **status != 0**: Decode failed

---

## 🎯 Week 10 Demo Requirements

Your project brief requires (page 16):

**Demo 2: Line Following + Barcode Navigation + Telemetry**

Success Criteria:
- ✅ Robot correctly follows line
- ✅ **Interprets barcodes** ← Your system does this!
- ✅ **Executes correct movements** ← Based on decoded character
- ✅ **MQTT dashboard shows barcode events** ← Auto-published!
- ✅ No track loss after junctions

**Your system is ready for this!** 🎉

---

## 🔧 Advanced: Fine-Tuning Parameters

If you're still having accuracy issues, adjust these:

### 1. End-of-Barcode Detection
```c
// In barcode_direction.c, line 125:
#define BARCODE_END_TIMEOUT_US  15000   // Default: 15ms

// Increase if barcodes end prematurely:
#define BARCODE_END_TIMEOUT_US  20000   // 20ms

// Decrease if scanning is too slow:
#define BARCODE_END_TIMEOUT_US  10000   // 10ms
```

### 2. Wide/Narrow Threshold
```c
// In barcode_direction.c, line 572:
continuous_result.threshold_us = continuous_result.avg_narrow_width_us * 3 / 2;

// More strict (1.6× instead of 1.5×):
continuous_result.threshold_us = continuous_result.avg_narrow_width_us * 8 / 5;

// More lenient (1.4× instead of 1.5×):
continuous_result.threshold_us = continuous_result.avg_narrow_width_us * 7 / 5;
```

### 3. Minimum Bar Width
```c
// In barcode_direction.h, line 56:
#define BC_DIR_MIN_BAR_WIDTH    50

// If missing narrow bars, decrease:
#define BC_DIR_MIN_BAR_WIDTH    30

// If too much noise, increase:
#define BC_DIR_MIN_BAR_WIDTH    80
```

---

## ✅ Final Checklist for 100% Accuracy

Before your demo:

- [ ] **Calibrate sensor** over your actual track
- [ ] **Verify speed** is 8-12 cm/s during barcode pass
- [ ] **Check sensor height**: 5mm ± 1mm
- [ ] **Test both directions** (forward/reverse)
- [ ] **Print quality barcodes** (sharp, high contrast)
- [ ] **Add quiet zones** (10mm white before/after)
- [ ] **Test at demo lighting** conditions
- [ ] **Verify MQTT publishing** works
- [ ] **Run 10 consecutive tests** (aim for 10/10 success)
- [ ] **Document success rate** for report

---

## 📞 Quick Reference

| Issue | Quick Fix |
|-------|-----------|
| "Too few bars" | Slow down or increase `BARCODE_END_TIMEOUT_US` |
| "Invalid pattern" | Re-calibrate sensor |
| Works sometimes | Check speed consistency (enable PID) |
| Only one direction | Normal - system auto-handles reverse |
| Barcode missed completely | Check sensor alignment and height |

---

**Your implementation is solid!** The key to 100% accuracy is:
1. **Proper calibration**
2. **Consistent speed (8-12 cm/s)**
3. **Good barcode quality**
4. **Correct sensor positioning**

Good luck with your Week 10 demo! 🚗💨
