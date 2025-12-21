# TeamCode OpModes Directory

This directory contains custom OpModes (robot programs) for your FTC team. OpModes are programs that run during either the autonomous or driver-controlled (teleop) periods of an FTC match.

## 📁 Directory Structure

```
teamcode/
├── auto/          - Autonomous sample OpModes (all disabled)
├── basic/         - Basic driving examples (1 enabled)
├── concept/       - Vision and concept demonstrations (all disabled)
├── pickle/        - 🏆 TEAM COMPETITION ROBOT (both enabled)
├── sensor/        - Sensor usage examples (all disabled)
├── starter/       - StarterBot reference examples (all disabled)
└── tele/          - Teleop driving samples (all disabled)
```

---

## 🏆 COMPETITION ROBOT - Pickle Bot

### **pickle/PickleTeleOp.java** ✅ ENABLED
**Type:** TeleOp (Driver-Controlled)
**Display Name:** "PickleTeleOp"
**Package:** `org.firstinspires.ftc.teamcode.pickle`

**Purpose:** Team's main competition driver control program for DECODE (2025-2026) season

**Features:**
- **4-motor mecanum drive** with full omnidirectional movement (strafe, rotate, drive)
- High-speed launcher motor with velocity control
- Two continuous rotation servos for feeding projectiles
- State machine for launch sequence management
- **Auto-align to goal** using IMU (Left Bumper toggles ON/OFF)
- Alliance selection (A=Red, B=Blue) during init
- BRAKE mode for precise stopping
- RUN_WITHOUT_ENCODER for maximum drive speed (direct power control)
- RUN_USING_ENCODER on launcher for velocity control
- Custom PIDF coefficients for launcher optimization
- Linear input response for maximum responsiveness
- Scaled deadband to eliminate joystick drift

**Speed Settings:**
- Drive Speed: **100%** power (full speed for competition)
- Launcher: 1,100 ticks/sec target velocity (~40% of max)

**Motor Direction Configuration:**
- Left motors (front_left, back_left): `FORWARD`
- Right motors (front_right, back_right): `REVERSE`

**Controls:**
- Left Stick Y: Forward/Backward
- Left Stick X: Strafe (side-to-side)
- Right Stick X: Rotation (cancels auto-align if active)
- Left Bumper: Toggle auto-align to goal ON/OFF
- Y Button (hold): Pre-spin launcher to target velocity
- X Button (hold): Stop launcher / cancel launch sequence
- A Button: Select RED alliance (during init)
- B Button: Select BLUE alliance (during init)
- Right Bumper: Fire projectile (spin up + feed when ready)

**Hardware Requirements:**
- Motors: `front_left`, `front_right`, `back_left`, `back_right`, `launcher`
- Servos: `left_feeder`, `right_feeder`
- IMU: `imu` (built into Control Hub)

---

### **pickle/PickleAutoOp.java** ✅ ENABLED
**Type:** Autonomous
**Display Name:** "PickleAutoOp"
**Package:** `org.firstinspires.ftc.teamcode.pickle`

**Purpose:** Team's primary competition autonomous routine (encoder-based)

**What It Does:**
1. Optional start delay (for partner coordination)
2. Drive clear of starting corner
3. Turn toward goal (mirrored by alliance)
4. Drive to shooting position
5. Align with goal using AprilTag (if visible)
6. Launch 3 projectiles
7. Park for TeleOp-friendly position

**Speed Settings:**
- Drive Speed: **80%** power
- Rotate Speed: 20% power
- Approach Speed: 30% power (near goal)

**Motor Direction Configuration:** Same as PickleTeleOp

**Hardware Requirements:** Same as PickleTeleOp + Webcam

---

### **pickle/PickleAutoHolonomic.java** ✅ ENABLED
**Type:** Autonomous
**Display Name:** "PickleAutoHolonomic"
**Package:** `org.firstinspires.ftc.teamcode.pickle`

**Purpose:** Advanced autonomous with sensor fusion (odometry + AprilTag)

**What It Does:**
- Uses encoder-based odometry for continuous position tracking
- AprilTag corrections for absolute positioning
- Road Runner-style path following with waypoints
- Smooth heading control while driving

**Speed Settings:**
- Max Drive Speed: **80%** power
- Approach Speed: 35% power
- Align Speed: 25% power

**Motor Direction Configuration:** Same as PickleTeleOp

**Hardware Requirements:** Same as PickleAutoOp

---

## 📚 STARTERBOT REFERENCE EXAMPLES

These are the official FTC StarterBot examples. Your team's competition robot (Pickle Bot) is based on these files.

### **starter/StarterBotTeleop.java** 🔒 DISABLED
**Type:** TeleOp (Reference)
**Display Name:** Would be "StarterBotTeleop" if enabled
**Package:** `org.firstinspires.ftc.teamcode.starter`

**Purpose:** Official StarterBot reference implementation for DECODE season

**Features:** Same as PickleTeleOp (this is the original template)
- 2-motor tank drive + launcher + feeders
- Arcade-style controls
- State machine for launches
- Velocity-controlled launcher

**Note:** This is disabled because your team uses the customized version in `pickle/PickleTeleOp.java`

---

### **starter/StarterBotAuto.java** 🔒 DISABLED
**Type:** Autonomous (Reference)
**Display Name:** Would be "StarterBotAuto" if enabled
**Package:** `org.firstinspires.ftc.teamcode.starter`

**Purpose:** Official StarterBot autonomous reference implementation

**What It Does:** Same as PickleAutoOp (this is the original template)
- Launch projectiles and drive away from start

**Note:** This is disabled because your team uses the customized version in `pickle/PickleAutoOp.java`

---

## 🎓 SAMPLE/LEARNING OPMODES

### **basic/BasicOmniOpMode_Linear.java** ✅ ENABLED
**Type:** TeleOp (Enhanced 2-Motor Drive)
**Hardware:** 2-motor tank drive
**Purpose:** Production-ready, enhanced 2-motor drive with advanced features

**Features:**
- ✨ **NEW: Competition-hardened with code review fixes**
- Try-catch error handling for graceful failures
- Null checks and hardware validation
- Joystick deadzone (5%) to prevent drift
- Rate-limited telemetry (100ms updates)
- Encoder availability auto-detection with fallback
- 3-speed control system:
  - **Slow Mode (30%)** - Left Bumper
  - **Normal Mode (80%)** - Default
  - **Turbo Mode (100%)** - Right Bumper
- BRAKE mode for precise control
- RUN_USING_ENCODER for velocity control
- Enhanced telemetry with motor velocities
- Arcade-style controls

**Controls:**
- Left Stick Y: Forward/Backward
- Right Stick X: Rotation
- Left Bumper: Slow mode (precision)
- Right Bumper: Turbo mode (speed)

**Hardware Requirements:**
- Motors: `left_drive`, `right_drive`

**Note:** Works with mecanum wheels but provides tank drive only (no strafing). Upgrade to 4 motors for full omnidirectional movement.

**Code Quality:** Grade A - Production-ready for competition ✓

---

### **basic/BasicOpMode_Linear.java** 🔒 DISABLED
**Type:** TeleOp (Basic Sample)
**Hardware:** 2-motor tank drive
**Purpose:** Minimal skeleton example for learning LinearOpMode structure

**What It Does:**
- Basic 2-motor tank drive
- Simple joystick mapping (left stick = left motor, right stick = right motor)
- Demonstrates LinearOpMode basics

**Best For:** Learning the fundamental structure of an OpMode

---

### **basic/BasicOpMode_Iterative.java** 🔒 DISABLED
**Type:** TeleOp (Iterative Sample)
**Hardware:** 2-motor tank drive
**Purpose:** Minimal skeleton example for learning iterative OpMode structure

**What It Does:**
- Basic 2-motor tank drive using OpMode base class
- Demonstrates init() and loop() methods
- Simpler alternative to LinearOpMode

**Best For:** Learning iterative programming style

---

## 🎮 TELEOP SAMPLES

### **tele/RobotTeleopPOV_Linear.java** 🔒 DISABLED
**Type:** TeleOp (POV Style)
**Package:** `org.firstinspires.ftc.teamcode.tele`
**Hardware:** 2-motor drive + arm motor + claw servos

**Purpose:** Demonstrates POV (Point of View) control style with manipulator

**What It Does:**
- POV arcade drive (left stick forward/back, right stick rotate)
- Arm control using Y/A buttons (raise/lower)
- Claw control using bumpers (open/close slowly)
- Includes manipulator example

**Best For:** Learning how to control arms and claws

---

### **tele/RobotTeleopTank_Iterative.java** 🔒 DISABLED
**Type:** TeleOp (Tank Style)
**Package:** `org.firstinspires.ftc.teamcode.tele`
**Hardware:** 4-motor drive

**Purpose:** Demonstrates tank-style controls using iterative OpMode

**What It Does:**
- Tank drive (left stick = left side, right stick = right side)
- Uses OpMode (iterative) instead of LinearOpMode
- Direct motor control

**Best For:** Learning tank drive controls and iterative OpMode structure

---

## 🤖 AUTONOMOUS SAMPLES

### **auto/RobotAutoDriveByTime_Linear.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 4-motor drive
**Purpose:** Simple time-based autonomous driving

**What It Does:**
- Drives forward for a set time
- Turns for a set time
- Drives backward for a set time
- Uses `sleep()` for timing

**Best For:** Simplest autonomous introduction, no encoders needed

---

### **auto/RobotAutoDriveByEncoder_Linear.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 4-motor drive with encoders
**Purpose:** Precise encoder-based autonomous driving

**What It Does:**
- Drives specific distances using encoder counts
- Calculates inches/centimeters from wheel diameter
- More accurate than time-based driving
- Includes functions for driving straight and turning

**Best For:** Learning encoder-based autonomous programming

---

### **auto/RobotAutoDriveToLine_Linear.java** 🔒 DISABLED
**Type:** Autonomous
**Package:** `org.firstinspires.ftc.teamcode.auto`
**Hardware:** Drive motors + color sensor

**Purpose:** Autonomous driving to a colored line using color sensor

**What It Does:**
- Drives forward until color sensor detects a line
- Uses color sensor threshold detection
- Stops when target color is found

**Best For:** Learning sensor-based autonomous navigation

---

### **auto/RobotAutoDriveToAprilTagTank.java** 🔒 DISABLED
**Type:** Autonomous
**Hardware:** 2-motor tank drive + camera
**Purpose:** Vision-based autonomous using AprilTags

**What It Does:**
- Uses camera to detect AprilTags
- Drives to a specific AprilTag location
- Adjusts position based on tag detection
- Demonstrates vision processing

**Best For:** Learning AprilTag detection and vision-based navigation

**Hardware Requirements:**
- Camera configured in Robot Configuration
- Motors: `left_drive`, `right_drive`

---

### **auto/RobotAutoDriveToAprilTagOmni.java** 🔒 DISABLED
**Type:** Autonomous
**Package:** `org.firstinspires.ftc.teamcode.auto`
**Hardware:** 4-motor omni/mecanum drive + camera

**Purpose:** Vision-based autonomous using AprilTags with omnidirectional drive

**What It Does:**
- Uses camera to detect AprilTags
- Drives and strafes to a specific AprilTag location
- Leverages omnidirectional capabilities
- Demonstrates vision processing with mecanum/omni wheels

**Best For:** Learning AprilTag detection with full mobility

**Hardware Requirements:**
- Camera configured in Robot Configuration
- Motors: 4-motor omnidirectional configuration

---

## 📷 VISION & CONCEPT SAMPLES

### **concept/ConceptAprilTagEasy.java** 🔒 DISABLED
**Type:** TeleOp (Vision Demonstration)
**Hardware:** Camera (any FTC-approved camera)
**Purpose:** Simple AprilTag detection example for DECODE season

**What It Does:**
- Detects AprilTags in camera view
- Displays tag ID, position, and orientation
- Shows how to use the AprilTag library
- Demonstrates basic vision processing

**Best For:** Learning AprilTag detection basics

**Note:** Uses DECODE (2025-2026) season AprilTag library

---

## 🔬 SENSOR SAMPLES

### **sensor/SensorTouch.java** ✅ ENABLED
**Type:** TeleOp (Sensor Demo)
**Display Name:** "Sensor: REV touch sensor"
**Package:** `org.firstinspires.ftc.teamcode.sensor`
**Hardware:** Touch sensor or magnetic limit switch

**Purpose:** Demonstrates how to use REV Touch Sensor or Magnetic Limit Switch

**What It Does:**
- Reads touch sensor state (pressed/not pressed)
- Displays sensor status on telemetry
- Works with any "active low" touch sensor

**Hardware Requirements:**
- Touch sensor named `sensor_touch` in configuration
- REV Touch Sensor must be on digital port 1, 3, 5, or 7
- Magnetic Limit Switch can use any digital port

**Best For:** Learning how to read digital sensors

---

### **sensor/SensorColor.java** ✅ ENABLED
**Type:** TeleOp (Sensor Demo)
**Display Name:** "Sensor: REV color sensor"
**Package:** `org.firstinspires.ftc.teamcode.sensor`
**Hardware:** Color sensor (REV or compatible)

**Purpose:** Demonstrates how to use a color sensor

**What It Does:**
- Reads RGB color values
- Measures distance (if sensor supports it)
- Adjustable gain for different lighting conditions
- Changes robot controller LED to match detected color

**Hardware Requirements:**
- Color sensor named `sensor_color` in configuration
- Works with REV Color Sensor V3 or compatible NormalizedColorSensor

**Best For:** Learning color detection for autonomous line following or object identification

---

## 🎮 Control Styles Explained

### Arcade Drive (Most Common)
- **Left Stick Y:** Forward/Backward
- **Right Stick X:** Rotation
- One stick moves, one stick turns
- **Used by:**
  - `pickle/PickleTeleOp.java` ✅ (Competition)
  - `basic/BasicOmniOpMode_Linear.java` ✅
  - `starter/StarterBotTeleop.java` 🔒
  - `tele/RobotTeleopPOV_Linear.java` 🔒

### Tank Drive
- **Left Stick Y:** Left side motors
- **Right Stick Y:** Right side motors
- Direct control of each side independently
- **Used by:**
  - `basic/BasicOpMode_Linear.java` 🔒
  - `tele/RobotTeleopTank_Iterative.java` 🔒

---

## 🔧 Hardware Configuration Guide

Before using any OpMode, configure your robot hardware in the **Robot Configuration** menu on the Driver Station:

### Pickle Bot Configuration (Competition Robot) 🏆
**Required for:** `pickle/PickleTeleOp.java`, `pickle/PickleAutoOp.java`, and `pickle/PickleAutoHolonomic.java`

```
Motors (Control Hub):
  - front_left   (goBILDA 312 RPM motor, Port 0)
  - front_right  (goBILDA 312 RPM motor, Port 1)
  - back_left    (goBILDA 312 RPM motor, Port 2)
  - back_right   (goBILDA 312 RPM motor, Port 3)
  - launcher     (goBILDA high-speed motor with encoder)

Servos:
  - left_feeder  (Continuous Rotation Servo)
  - right_feeder (Continuous Rotation Servo)

I2C:
  - imu (built into Control Hub, auto-configured)

Camera:
  - Webcam 1 (USB webcam for AprilTag detection)
```

**Motor Direction Configuration:**
```
Left motors (front_left, back_left):   FORWARD
Right motors (front_right, back_right): REVERSE
```

**Configuration Notes:**
- **4-motor mecanum drive** with full omnidirectional movement
- Drive motors use RUN_WITHOUT_ENCODER (encoders optional but recommended for odometry)
- Launcher motor must have encoder cable connected for velocity control
- Left feeder servo is set to REVERSE direction in code
- All drive motors use BRAKE mode for precise stopping
- IMU orientation: Logo LEFT, USB FORWARD (adjust if Control Hub mounted differently)

---

### Basic Drive Configuration (Practice/Testing)
**Required for:** `basic/BasicOmniOpMode_Linear.java` and basic samples

```
Motors:
  - left_drive (motor, port ?)
  - right_drive (motor, port ?)
```

---

### 4-Motor Drive Configuration (Advanced Samples)
**Required for:** Some autonomous samples

```
Motors:
  - left_front_drive
  - left_back_drive
  - right_front_drive
  - right_back_drive
```

---

### Sensor Configuration (Optional - For Sensor Samples)
**Required for:** `sensor/` samples

```
Sensors:
  - sensor_touch (REV Touch Sensor, digital port 1/3/5/7)
  - sensor_color (REV Color Sensor V3, I2C port)
```

---

## 📋 Quick Start Guide

### For Competition Day 🏆
1. **Driver Control:** Use `PickleTeleOp` - 4-motor mecanum with 100% power
2. **Autonomous (Reliable):** Use `PickleAutoOp` - encoder-based, 80% speed
3. **Autonomous (Advanced):** Use `PickleAutoHolonomic` - with AprilTag fusion
4. **Backup/Testing:** Use `BasicOmniOpMode_Linear` (drive only, no launcher)

### For Drivers (Practice)
1. **Full Robot:** `pickle/PickleTeleOp.java` ✅ (100% speed, Left Bumper for auto-align)
2. **Drive Only:** `basic/BasicOmniOpMode_Linear.java` ✅
3. **Learning:** Enable and try sample OpModes in `tele/` folder

### For Programmers
1. **Customize Competition Robot:** Edit files in `pickle/` folder
2. **Hardware Names:** See `pickle/config/PickleHardwareNames.java`
3. **Reference Examples:** Study `starter/` folder (original StarterBot code)
4. **Learn Basics:** Study `basic/` samples

### For Autonomous Development
1. **Primary Auto:** Customize `pickle/PickleAutoOp.java` (80% speed)
2. **Advanced Auto:** Study `pickle/PickleAutoHolonomic.java` (odometry + vision)
3. **Time-Based (Easiest):** Study `auto/RobotAutoDriveByTime_Linear.java`
4. **Encoder-Based (Better):** Study `auto/RobotAutoDriveByEncoder_Linear.java`
5. **Vision-Based (Advanced):** Study `auto/RobotAutoDriveToAprilTagTank.java`

---

## ✅ Enabled vs Disabled OpModes

**✅ ENABLED** (appears in Driver Station menu):
- `pickle/PickleTeleOp.java` - Competition teleop (4-motor mecanum, 100% speed)
- `pickle/PickleAutoOp.java` - Competition autonomous (encoder-based, 80% speed)
- `pickle/PickleAutoHolonomic.java` - Advanced autonomous (odometry + AprilTag fusion)
- `basic/BasicOmniOpMode_Linear.java` - Enhanced 2-motor drive with speed modes
- `sensor/SensorTouch.java` - Touch sensor demo
- `sensor/SensorColor.java` - Color sensor demo

**🔒 DISABLED** (commented with `@Disabled`, hidden from menu):
- `starter/` - Both StarterBot reference files
- `auto/` - All 5 autonomous samples
- `basic/` - 2 basic samples (BasicOpMode_Linear, BasicOpMode_Iterative)
- `tele/` - Both teleop samples
- `concept/` - AprilTag concept demo

### How to Enable/Disable OpModes

**To enable a disabled OpMode:**
1. Open the file in Android Studio
2. Find the line: `@Disabled`
3. Comment it out: `//@Disabled` or delete the line
4. Rebuild and deploy: `./gradlew installDebug`

**To disable an OpMode:**
1. Add `@Disabled` annotation before `@TeleOp` or `@Autonomous`
```java
@Disabled
@TeleOp(name = "My OpMode")
public class MyOpMode extends LinearOpMode {
```

---

## 🏆 Competition Strategy

### Driver-Controlled Period
**Primary:** `pickle/PickleTeleOp` ✅ - Full 4-motor mecanum drive with launcher (100% speed)
**Backup:** `basic/BasicOmniOpMode_Linear` ✅ - Drive only (if launcher fails)

**Driver Tips:**
- Press **Left Bumper** to toggle auto-align ON/OFF
- Move right stick to cancel auto-align and regain manual control
- Hold **Y** to pre-spin launcher before firing
- Hold **X** to stop launcher or cancel a shot
- Select alliance during init: **A** = Red, **B** = Blue

### Autonomous Period
**Primary:** `pickle/PickleAutoOp` ✅ - Reliable encoder-based autonomous (80% speed)
**Advanced:** `pickle/PickleAutoHolonomic` ✅ - Sensor fusion with AprilTag corrections

**When to use which:**
- `PickleAutoOp` - More predictable, good for qualification matches
- `PickleAutoHolonomic` - More accurate positioning, better for elimination rounds

---

## 📚 Additional Resources

- **Official FTC Docs:** https://ftc-docs.firstinspires.org/
- **Game Manual 0:** https://gm0.org/ (community guide)
- **JavaDoc Reference:** https://javadoc.io/doc/org.firstinspires.ftc
- **Sample OpModes:** `FtcRobotController/src/main/java/.../samples/`

---

## 🛠️ Development Tips

1. **Always test motor directions** - Push left stick forward, robot should go forward
2. **Use telemetry** - Display debug info on Driver Station
3. **Start simple** - Get basic movement working before adding complexity
4. **Use encoders** - RUN_USING_ENCODER provides better control
5. **Enable BRAKE mode** - Stops robot faster and more precisely
6. **Add error handling** - Check for null hardware and configuration errors
7. **Rate-limit telemetry** - Update every 100ms, not every loop

---

## 🐛 Common Issues

**OpMode doesn't appear in Driver Station:**
- Check that `@Disabled` is removed or commented out
- Verify the OpMode compiled successfully
- Rebuild and reinstall the app

**Robot doesn't move:**
- Check motor configuration names match code
- Verify motors are plugged in
- Test motor directions individually

**Motors run backward / Robot drives in reverse:**
- Swap ALL motor directions: `FORWARD` ↔ `REVERSE` for all 4 motors
- Current Pickle Bot config: Left=FORWARD, Right=REVERSE
- If only one side is wrong, swap just that side
- See `PickleHardwareNames.java` for hardware name reference

**Controller drift:**
- Use deadzone (built into Enhanced 2-Motor Drive)
- Recalibrate gamepad in Driver Station

**Encoders not working:**
- Check encoder cables are plugged in adjacent to motor
- Verify motor polarity is consistent
- Use Enhanced 2-Motor Drive (auto-detects encoders)

---

## 📝 License

All OpModes in this directory are based on FTC SDK samples and are subject to the FIRST Tech Challenge software license. See individual file headers for specific copyright information.

---

## 📂 File Organization Summary

| Folder | Purpose | Status | File Count |
|--------|---------|--------|------------|
| **pickle/** | 🏆 Team competition robot | 3 enabled | 3+ files |
| **pickle/config/** | Hardware configuration constants | Support files | 1 file |
| **pickle/drive/** | Mecanum drive helper classes | Support files | 1 file |
| **pickle/field/** | Field constants and alliance info | Support files | 3 files |
| **pickle/geometry/** | Pose and translation classes | Support files | 3 files |
| **pickle/odometry/** | Encoder-based position tracking | Support files | 1 file |
| **pickle/path/** | Path following classes | Support files | 2 files |
| **pickle/vision/** | AprilTag localization | Support files | 1 file |
| **starter/** | Reference StarterBot examples | All disabled | 2 files |
| **basic/** | Basic driving samples | 1 enabled | 3 files |
| **auto/** | Autonomous samples | All disabled | 5 files |
| **tele/** | Teleop driving samples | All disabled | 2 files |
| **sensor/** | Sensor usage demos | 2 enabled | 2 files |
| **concept/** | Vision & concepts | All disabled | 1 file |

**Total:** 25+ Java files organized across multiple packages

---

**Last Updated:** 2025-12-20
**Season:** DECODE (2025-2026)
**SDK Version:** 11.0
**Team Robot:** Pickle Bot (based on goBILDA StarterBot)
