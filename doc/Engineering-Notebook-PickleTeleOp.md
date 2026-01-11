# Engineering Notebook: PickleTeleOp.java Development History

**Team Pickle - FTC 2025-2026 DECODE Season**

This document traces the complete software development journey of our TeleOp control system, documenting 28 commits over approximately 2 months of iterative development.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Development Timeline](#development-timeline)
3. [Phase 1: Foundation (Nov 2-16)](#phase-1-foundation-nov-2-16)
4. [Phase 2: Drive System Upgrade (Nov 16 - Dec 14)](#phase-2-drive-system-upgrade-nov-16---dec-14)
5. [Phase 3: Advanced Features (Dec 14-21)](#phase-3-advanced-features-dec-14-21)
6. [Phase 4: Competition Tuning (Jan 4)](#phase-4-competition-tuning-jan-4)
7. [Key Technical Decisions](#key-technical-decisions)
8. [Lessons Learned](#lessons-learned)

---

## Project Overview

### Starting Point
- **Base Code:** GoBilda StarterBotTeleop (2-motor tank drive)
- **Control Scheme:** Arcade drive (left stick forward, right stick rotate)
- **Launcher:** 4-state machine, single shot per button press

### Final Result
- **Drive System:** 4-motor Mecanum (full omnidirectional control)
- **Control Scheme:** Mecanum drive with deadband, input shaping, strafe compensation
- **Launcher:** 5-state machine, one-button 3-ball rapid-fire with velocity recovery
- **Extras:** IMU-based auto-align, alliance selection, comprehensive telemetry

### Metrics
- **Total Commits:** 28
- **Development Period:** November 2, 2025 - January 4, 2026
- **Lines of Code:** ~1,100 (final)

---

## Development Timeline

```
Nov 2   ████ Initial starter code
Nov 9   ██   Add mecanum notes
Nov 10  ██   Disable starter, prepare for customization
Nov 16  ████████████ Speed control, deadband, slow mode, cooldown
Nov 28  ██   AprilTag integration
Dec 14  ████████████████████ Major mecanum implementation + auto-align
Dec 15  ████ Auto-align tuning
Dec 20  ████ Competition tuning
Dec 21  ████ Heading constants
Jan 4   ████████ Multi-shot + velocity recovery (FINAL)
```

---

## Phase 1: Foundation (Nov 2-16)

### Commit: Initial Starter Code
**Date:** November 2, 2025
**Hash:** `ffdb225`

**What We Did:**
- Added GoBilda's StarterBotTeleop as our starting point
- 2-motor tank drive with arcade control
- Basic 4-state launcher (IDLE → SPIN_UP → LAUNCH → LAUNCHING)

**Original Code Structure:**
```java
// Original: 2 motors, tank drive
private DcMotor leftDrive = null;
private DcMotor rightDrive = null;

void arcadeDrive(double forward, double rotate) {
    leftPower = forward + rotate;
    rightPower = forward - rotate;
    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
}
```

---

### Commit: Add Speed Control
**Date:** November 16, 2025
**Hash:** `4aa6b03`

**Problem Identified:**
- Full speed was too fast to control accurately
- Needed precision mode for alignment and delicate maneuvers

**Solution:**
```java
// Speed control constants
final double NORMAL_DRIVE_SPEED = 0.7;  // 70% speed
final double SLOW_DRIVE_SPEED = 0.3;    // 30% for precision

// Toggle slow mode with left bumper
if (gamepad1.left_bumper) {
    slowMode = true;
} else {
    slowMode = false;
}

double speedMultiplier = slowMode ? SLOW_DRIVE_SPEED : NORMAL_DRIVE_SPEED;
leftPower = (forward + rotate) * speedMultiplier;
```

**Result:** Much easier to control, especially during alignment.

---

### Commit: Add Launch Cooldown
**Date:** November 16, 2025
**Hash:** `0cefe69`

**Problem Identified:**
- Rapid button pressing could fire multiple balls unexpectedly
- Needed controlled timing between launches

**Solution:**
- Added 2-second cooldown timer between launches
- Prevented accidental multi-fire

---

### Commit: Refactor Arcade Drive with Deadband
**Date:** November 16, 2025
**Hash:** `b1f417d`

**Problem Identified:**
- Joystick drift caused robot to move when not touching controls
- Small values like 0.02 or -0.03 from stick noise

**Solution - Deadband:**
```java
double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
        return 0.0;  // Ignore noise
    }
    return value;
}
```

**Problem Identified #2:**
- When forward + rotate exceeded 1.0, motor clipping distorted turn radius
- Example: forward=1.0, rotate=1.0 → left=2.0 (clipped to 1.0), right=0.0

**Solution - Normalization:**
```java
// Find max and scale proportionally
double maxPower = Math.max(1.0, Math.max(Math.abs(rawLeftPower), Math.abs(rawRightPower)));
rawLeftPower = rawLeftPower / maxPower;
rawRightPower = rawRightPower / maxPower;
```

**Result:** Smooth, predictable control with no drift or distortion.

---

## Phase 2: Drive System Upgrade (Nov 16 - Dec 14)

### Commit: Restructure for Pickle Robot
**Date:** November 16, 2025
**Hash:** `6c23922`, `bd19cbc`

**What We Did:**
- Renamed OpMode from StarterBot to Pickle
- Prepared codebase for hardware upgrade to Mecanum wheels

---

### Commit: Add Strafe Compensation
**Date:** December 14, 2025
**Hash:** `68014db`

**Problem Identified:**
- After installing Mecanum wheels, strafing felt sluggish compared to forward movement
- This is normal due to roller friction physics

**Solution:**
```java
/*
 * STRAFE COMPENSATION MULTIPLIER
 *
 * Mecanum wheels are less efficient at strafing due to:
 * - Roller friction (rollers slide sideways)
 * - Weight transfer (robot "leans")
 * - Floor surface interaction
 */
final double STRAFE_MULTIPLIER = 1.1;

// Applied in mecanumDrive()
strafe = strafe * STRAFE_MULTIPLIER;
```

**Tuning Notes:**
- Started at 1.0 (no compensation)
- Tested 1.2 (too aggressive)
- Settled on 1.1 (feels balanced)

---

### Commit: Implement Full Mecanum Drive System
**Date:** December 14, 2025
**Hash:** `ac028b7`

**Major Change:** Complete rewrite from 2-motor tank to 4-motor mecanum

**Before (Tank Drive):**
```java
private DcMotor leftDrive = null;
private DcMotor rightDrive = null;

leftPower = forward + rotate;
rightPower = forward - rotate;
```

**After (Mecanum Drive):**
```java
private DcMotor frontLeft = null;
private DcMotor frontRight = null;
private DcMotor backLeft = null;
private DcMotor backRight = null;

// Mecanum formula
double rawFrontLeft = forward + strafe + rotate;
double rawFrontRight = forward - strafe - rotate;
double rawBackLeft = forward - strafe + rotate;
double rawBackRight = forward + strafe - rotate;
```

**Motor Direction Configuration:**
```java
// Left side: FORWARD, Right side: REVERSE
frontLeft.setDirection(DcMotor.Direction.FORWARD);
backLeft.setDirection(DcMotor.Direction.FORWARD);
frontRight.setDirection(DcMotor.Direction.REVERSE);
backRight.setDirection(DcMotor.Direction.REVERSE);
```

**Additional Changes:**
- Increased speed from 70% to 100% for competition
- Reduced launch cooldown from 2.0s to 1.0s
- Updated telemetry to show all 4 motor powers

**Result:** Full omnidirectional movement - forward, backward, strafe, rotate, and any combination!

---

## Phase 3: Advanced Features (Dec 14-21)

### Commit: Add IMU and Auto-Align Feature
**Date:** December 15, 2025
**Hash:** `8df74ff`

**Feature:** Automatic rotation to face goal at optimal angle

**Implementation:**
```java
// IMU initialization
imu = hardwareMap.get(IMU.class, "imu");
RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
imu.initialize(new IMU.Parameters(orientation));

// Auto-align state machine
private enum AlignState {
    IDLE,
    ALIGNING,
    ALIGNED,
}

// Target headings (perpendicular to goal ramps)
final double RED_GOAL_PERPENDICULAR_HEADING_DEG = -45.0;   // Turn RIGHT
final double BLUE_GOAL_PERPENDICULAR_HEADING_DEG = 45.0;   // Turn LEFT
```

**Proportional Control Algorithm:**
```java
double getAlignRotation() {
    double headingErrorDeg = normalizeAngleDegrees(targetHeadingDeg - cachedHeadingDeg);

    if (Math.abs(headingErrorDeg) <= ALIGN_TOLERANCE_DEG) {
        alignState = AlignState.ALIGNED;
        return 0.0;
    }

    // P-controller: rotation power proportional to error
    double rotatePower = -headingErrorDeg * ALIGN_HEADING_KP;
    rotatePower = clamp(rotatePower, -ALIGN_ROTATION_SPEED, ALIGN_ROTATION_SPEED);

    return rotatePower;
}
```

**Key Design Decision:**
- Auto-rotation bypasses input shaping (deadband, quadratic)
- Driver inputs still get shaped for fine control
- This prevents computed values from being too weak after squaring

---

### Commit: Update Hub Orientation
**Date:** December 14, 2025
**Hash:** `7b799ef`

**Problem:** Robot was turning the wrong direction during auto-align

**Root Cause:** Control Hub was mounted with logo facing LEFT, not UP

**Fix:**
```java
// Before
RevHubOrientationOnRobot.LogoFacingDirection.UP

// After
RevHubOrientationOnRobot.LogoFacingDirection.LEFT
```

**Lesson:** Always verify physical hardware orientation matches software configuration!

---

### Commit: Tune Auto-Align Headings
**Date:** December 21, 2025
**Hash:** `867d290`

**Testing Process:**
1. Manually aligned robot to face each goal
2. Read IMU heading from telemetry
3. Updated target constants to match actual field geometry

**Final Values:**
```java
final double RED_GOAL_PERPENDICULAR_HEADING_DEG = -45.0;
final double BLUE_GOAL_PERPENDICULAR_HEADING_DEG = 45.0;
final double ALIGN_TOLERANCE_DEG = 3.0;
final double ALIGN_ROTATION_SPEED = 0.8;
final double ALIGN_HEADING_KP = 0.015;
```

---

## Phase 4: Competition Tuning (Jan 4)

### Commit: Add Multi-Shot Sequence
**Date:** January 4, 2026
**Hash:** `58900f3`

**Problem Identified:**
- Pressing button 3 times for 3 balls was slow and error-prone
- Driver focus should be on positioning, not button timing

**Solution: One-Button 3-Ball Launch**

**New State Machine:**
```java
private enum LaunchState {
    IDLE,
    SPIN_UP,
    LAUNCH,
    LAUNCHING,
    WAIT_BETWEEN,  // NEW: Pause between shots
}

final int BALLS_PER_LAUNCH = 3;
final double SHOT_INTERVAL_SECONDS = 0.30;
int shotsFired = 0;
```

**Sequence Flow:**
```
Button Press
    ↓
IDLE → SPIN_UP (wait for velocity)
           ↓
       LAUNCH (activate feeders)
           ↓
       LAUNCHING (wait FEED_TIME)
           ↓
       shotsFired++
           ↓
       shotsFired < 3? → WAIT_BETWEEN → LAUNCH (loop)
           ↓
       shotsFired >= 3? → IDLE (done)
```

**Result:** Press once, fire all 3 balls with consistent timing!

---

### Commit: Implement Velocity Recovery
**Date:** January 4, 2026
**Hash:** `362614b`

**Problem Identified:**
- After each ball launch, motor velocity dropped
- Subsequent balls launched with less power, causing inconsistent trajectories

**Solution: Velocity Recovery Check**

```java
final double SPIN_UP_TIMEOUT_SECONDS = 1.0;
ElapsedTime spinUpTimer = new ElapsedTime();
boolean spinUpTimedOut = false;

case SPIN_UP:
    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

    boolean velocityReady = launcher.getVelocity() > LAUNCHER_MIN_VELOCITY;
    boolean timedOut = spinUpTimer.seconds() > SPIN_UP_TIMEOUT_SECONDS;

    if (velocityReady || timedOut) {
        spinUpTimedOut = timedOut && !velocityReady;
        launchState = LaunchState.LAUNCH;
    }
    break;

case LAUNCH:
    // Re-check velocity before each shot
    if (!spinUpTimedOut && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
        spinUpTimer.reset();
        launchState = LaunchState.SPIN_UP;  // Wait for recovery
        break;
    }
    // ... proceed with launch
```

**Timing Adjustments:**
```java
// Before
final double FEED_TIME_SECONDS = 0.20;

// After (longer feed time for reliable ball engagement)
final double FEED_TIME_SECONDS = 0.35;
```

**Telemetry for Debugging:**
```java
telemetry.addData("Spin-up Progress", "%.2fs | %.0f / %.0f",
    spinUpTimer.seconds(), launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);

telemetry.addData("Velocity Recovery", "%.0f / %.0f",
    launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
```

---

## Key Technical Decisions

### 1. Scaled Deadband vs Simple Deadband

**Simple Deadband Problem:**
- Creates a "jump" at threshold (0 → 0.05 instantly)
- Feels jarring to driver

**Scaled Deadband Solution:**
```java
double applyScaledDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
        return 0.0;
    }
    // Remap [deadband, 1.0] to [0.0, 1.0]
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
}
```

### 2. Separate Auto-Rotation from Driver Rotation

**Problem:** Input shaping (squaring) made auto-rotate too weak
- Auto-rotate of 0.12 became 0.12² = 0.0144

**Solution:**
```java
void mecanumDrive(double forward, double strafe, double driverRotate, double autoRotate) {
    // Shape driver inputs
    driverRotate = shapeInput(driverRotate);

    // Don't shape auto-rotate!
    double rotate = driverRotate + autoRotate;
}
```

### 3. State Machine for Launcher Control

**Why State Machine?**
- Avoids blocking loops (while/wait)
- Allows loop() to continue running (telemetry, drive, etc.)
- Easy to add new states (WAIT_BETWEEN)
- Clear, readable logic flow

### 4. Velocity-Based Launch Timing

**Why Not Time-Based?**
- Motor speed varies with battery voltage
- Load (ball resistance) affects spin-up time
- Encoder feedback provides actual readiness

**Implementation:**
```java
if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
    // Ready to fire
}
```

---

## Lessons Learned

### 1. Hardware Changes Require Software Updates
When we upgraded from tank to Mecanum, we had to:
- Add 2 more motor variables
- Rewrite entire drive algorithm
- Update motor direction configuration
- Tune new parameters (strafe compensation)

### 2. Physical Mounting Matters
IMU orientation must match software configuration. Our Control Hub was mounted with logo facing LEFT, not UP - caused reversed auto-align until fixed.

### 3. Test on Real Field
Auto-align headings had to be tuned on actual field:
- Theoretical values (45°) needed adjustment
- Telemetry was essential for reading actual headings

### 4. Velocity Recovery is Critical
After each ball launch, motor velocity drops. Without recovery check, subsequent balls were weak and inconsistent.

### 5. One-Button Operations Reduce Driver Cognitive Load
Driver can focus on positioning while robot handles 3-ball sequence automatically.

### 6. Comprehensive Telemetry Saves Time
Adding detailed telemetry (motor speeds, state machine status, heading values) made debugging much faster.

---

## Commit Log Summary

| Date | Hash | Description | Phase |
|------|------|-------------|-------|
| Nov 2 | `ffdb225` | Initial starter code | Foundation |
| Nov 9 | `bdf289b` | Add mecanum wheel notes | Foundation |
| Nov 10 | `1189e89` | Disable starter bot | Foundation |
| Nov 16 | `6c23922` | Restructure for Pickle | Foundation |
| Nov 16 | `bd19cbc` | Rename to Pickle | Foundation |
| Nov 16 | `4aa6b03` | Add speed control | Foundation |
| Nov 16 | `0cefe69` | Add launch cooldown | Foundation |
| Nov 16 | `6411421` | Reset timers in init | Foundation |
| Nov 16 | `77d29b0` | Toggle slow mode | Foundation |
| Nov 16 | `ef55c05` | Refactor launcher config | Foundation |
| Nov 16 | `b1f417d` | Add deadband + normalization | Foundation |
| Nov 16 | `2c4c483` | Refactor slow mode | Foundation |
| Nov 28 | `d1d53b8` | AprilTag init | Foundation |
| Dec 14 | `88ecad5` | Disable unused OpModes | Drive Upgrade |
| Dec 14 | `68014db` | Add strafe compensation | Drive Upgrade |
| Dec 14 | `ce77759` | Goal-tag alignment | Drive Upgrade |
| Dec 14 | `1f2d445` | Update hardware names | Drive Upgrade |
| Dec 14 | `ee5a297` | Various updates | Drive Upgrade |
| Dec 14 | `ac028b7` | **Implement Mecanum drive** | Drive Upgrade |
| Dec 14 | `7b799ef` | Fix hub orientation | Advanced |
| Dec 14 | `eeea70f` | Competition speed mode | Advanced |
| Dec 15 | `e35cd95` | AprilTag integration | Advanced |
| Dec 15 | `8df74ff` | **Auto-align + IMU** | Advanced |
| Dec 20 | `d998a47` | Performance tuning | Competition |
| Dec 20 | `8c6023b` | Launcher control update | Competition |
| Dec 21 | `867d290` | Tune heading constants | Competition |
| Jan 4 | `58900f3` | **Multi-shot sequence** | Competition |
| Jan 4 | `362614b` | **Velocity recovery** | Competition |

---

## Final Code Statistics

- **File:** `PickleTeleOp.java`
- **Lines:** ~1,100
- **Methods:** 12
- **State Machines:** 2 (LaunchState, AlignState)
- **Constants:** 20+
- **Hardware Devices:** 8 (4 motors, 2 servos, 1 launcher motor, 1 IMU)

---

*Document generated from git history analysis*
*Last updated: January 10, 2026*
