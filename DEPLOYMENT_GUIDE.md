# Deployment Guide: GitHub to Raspberry Pi Pico

## 📥 Option 1: Pull Latest Changes (If You Already Have the Repo)

If you already have the repository on your computer:

```bash
# Navigate to your project directory
cd /path/to/Supercar

# Fetch the latest changes
git fetch origin

# Switch to the barcode detection branch
git checkout claude/barcode-direction-detection-01XzUBuL3Jg2Qv28UhRiw7xb

# Pull the latest changes
git pull origin claude/barcode-direction-detection-01XzUBuL3Jg2Qv28UhRiw7xb
```

---

## 📥 Option 2: Fresh Download from GitHub

If you're starting fresh on a new computer:

### Method A: Using Git Clone
```bash
# Clone the repository
git clone https://github.com/WenJiangg/Supercar.git

# Navigate into the project
cd Supercar

# Switch to the barcode detection branch
git checkout claude/barcode-direction-detection-01XzUBuL3Jg2Qv28UhRiw7xb
```

### Method B: Download ZIP from GitHub
1. Go to: https://github.com/WenJiangg/Supercar
2. Click the branch dropdown (currently shows "main")
3. Select `claude/barcode-direction-detection-01XzUBuL3Jg2Qv28UhRiw7xb`
4. Click the green "Code" button
5. Select "Download ZIP"
6. Extract the ZIP file to your desired location

---

## 🔧 Step 1: Set Up Pico SDK

### Option A: Let CMake Download SDK Automatically (Easiest)
When building (Step 2), use this command:
```bash
cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON
```

### Option B: Manual SDK Setup (More Control)
```bash
# Download Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable (add to ~/.bashrc or ~/.zshrc for persistence)
export PICO_SDK_PATH=/path/to/pico-sdk
```

**For Windows users:** Set environment variable via System Properties:
- Right-click "This PC" → Properties → Advanced system settings
- Environment Variables → New
- Variable name: `PICO_SDK_PATH`
- Variable value: `C:\path\to\pico-sdk`

---

## 🛠️ Step 2: Build the Project

### Linux/macOS:
```bash
# Navigate to your Supercar project
cd /path/to/Supercar

# Create build directory and configure
cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON

# Build the project
cmake --build build

# The .uf2 file will be in build/robot_controller.uf2
```

### Windows (Command Prompt):
```cmd
cd C:\path\to\Supercar

cmake -B build -G "NMake Makefiles" -DPICO_SDK_FETCH_FROM_GIT=ON

cmake --build build
```

### Windows (PowerShell):
```powershell
cd C:\path\to\Supercar

cmake -B build -G "MinGW Makefiles" -DPICO_SDK_FETCH_FROM_GIT=ON

cmake --build build
```

**Build output:** `build/robot_controller.uf2`

---

## 📤 Step 3: Upload to Raspberry Pi Pico

### Upload Process:

1. **Enter BOOTSEL Mode:**
   - Disconnect Pico from USB
   - Hold down the **BOOTSEL** button (white button on Pico)
   - While holding, plug Pico into USB
   - Release BOOTSEL button
   - Pico will appear as a USB drive named **RPI-RP2**

2. **Copy the .uf2 File:**
   - Open the RPI-RP2 drive
   - Drag and drop `build/robot_controller.uf2` onto the drive
   - **OR** copy via command line:
     ```bash
     # Linux/macOS
     cp build/robot_controller.uf2 /Volumes/RPI-RP2/

     # Windows
     copy build\robot_controller.uf2 D:\
     # (Replace D: with your RPI-RP2 drive letter)
     ```

3. **Automatic Reboot:**
   - The Pico will automatically reboot and start running your code
   - The RPI-RP2 drive will disappear
   - Your robot is now running!

---

## 🔍 Step 4: Verify It's Working

### Serial Monitor (Optional but Recommended):

**Using screen (Linux/macOS):**
```bash
# Find your Pico's serial port
ls /dev/tty.usb*  # macOS
ls /dev/ttyACM*   # Linux

# Connect to serial monitor
screen /dev/ttyACM0 115200  # Linux
screen /dev/tty.usbmodem* 115200  # macOS
```

**Using PuTTY (Windows):**
1. Open PuTTY
2. Select "Serial"
3. Set port to COM port (check Device Manager)
4. Set speed to 115200
5. Click "Open"

### What You Should See:
```
=== RoboCar Control System ===
✓ Robot controller initialized
✓ Continuous barcode scanning enabled on GPIO28
✓ MQTT Client initialized

Line following active...
```

### Test Barcode Detection:
1. Place your robot over a CODE39 barcode
2. Move it slowly across the barcode at ~10 cm/s
3. You should see:
   ```
   🔍 BARCODE DETECTED: "A" | MOVE RIGHT | Scan: FORWARD (L→R) | Speed: 9.2 cm/s (optimal)
   ```

### Check MQTT (via Node-RED or MQTT client):
Subscribe to topic: `robocar/barcode`

Expected message:
```json
{
    "barcode": "A",
    "direction": "MOVE RIGHT",
    "scan_dir": "FORWARD (L→R)",
    "move_dir": 1,
    "bar_count": 45
}
```

---

## 🐛 Troubleshooting

### Build Fails: "SDK location was not specified"
**Solution:** Use `-DPICO_SDK_FETCH_FROM_GIT=ON` flag:
```bash
cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON
```

### Build Fails: "No CMAKE_C_COMPILER found"
**Solution:** Install build tools:
- **Linux:** `sudo apt install build-essential cmake gcc-arm-none-eabi`
- **macOS:** `brew install cmake gcc-arm-embedded`
- **Windows:** Install [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

### Pico Not Detected in BOOTSEL Mode
**Solutions:**
- Try a different USB cable (some cables are power-only)
- Try a different USB port
- Verify you're holding BOOTSEL before plugging in

### No Serial Output
**Solutions:**
- Check baud rate is set to 115200
- Verify correct COM port / device path
- Try disconnecting and reconnecting USB

### Barcode Not Detected
**Solutions:**
1. Run calibration first (see BARCODE_ACCURACY_GUIDE.md)
2. Check sensor height (should be 5mm ± 1mm from barcode)
3. Verify sensor is on GPIO28
4. Check MQTT connection is working
5. Test with a high-quality printed barcode

---

## 📁 Project Structure After Download

```
Supercar/
├── robot_controller.c          # Main control loop (barcode integration here)
├── barcode_direction.c         # Barcode decoder implementation
├── barcode_direction.h         # Barcode API header
├── barcode_direction_example.c # Usage examples
├── CMakeLists.txt              # Build configuration
├── .gitignore                  # Git ignore rules
├── BARCODE_DIRECTION_README.md # API documentation
├── BARCODE_ACCURACY_GUIDE.md   # Accuracy optimization guide
├── DEPLOYMENT_GUIDE.md         # This file
└── build/                      # Build output (after cmake)
    └── robot_controller.uf2    # Final firmware file
```

---

## 🚀 Quick Reference Commands

### Complete Build and Upload (Linux/macOS):
```bash
# 1. Download
git clone https://github.com/WenJiangg/Supercar.git
cd Supercar
git checkout claude/barcode-direction-detection-01XzUBuL3Jg2Qv28UhRiw7xb

# 2. Build
cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON
cmake --build build

# 3. Upload (Pico in BOOTSEL mode)
cp build/robot_controller.uf2 /Volumes/RPI-RP2/

# Done! Your robot is running!
```

### Rebuild After Code Changes:
```bash
# Just rebuild (no need to re-run cmake)
cmake --build build

# Upload new firmware
# (Put Pico in BOOTSEL mode again)
cp build/robot_controller.uf2 /Volumes/RPI-RP2/
```

---

## 📋 Pre-Demo Checklist

Before your Week 10 demo:
- [ ] Code downloaded and built successfully
- [ ] Firmware uploaded to Pico
- [ ] Barcode sensor calibrated (`barcode_direction_calibrate()`)
- [ ] Sensor height verified (5mm ± 1mm)
- [ ] MQTT connection tested
- [ ] Barcode detection tested both directions
- [ ] Speed verified (8-12 cm/s during barcode pass)
- [ ] High-quality barcodes printed with quiet zones
- [ ] Serial monitor shows "optimal" speed indicator

---

## 💡 Tips

1. **Keep build/ directory:** After first build, you only need to rebuild, not reconfigure
2. **Test incrementally:** Upload and test after each change
3. **Use serial monitor:** Essential for debugging barcode detection
4. **Backup working .uf2:** Keep a copy of robot_controller.uf2 that works
5. **Check MQTT first:** Ensure your MQTT broker is running before testing

---

Good luck with your deployment! Your barcode system is ready to go! 🚗💨
