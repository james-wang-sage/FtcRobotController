# Mecanum Drive Tuning Guide

This guide will help you tune your mecanum drivetrain for optimal performance. It covers motor setup, control response tuning, and troubleshooting common issues.

## Table of Contents

- [Understanding Mecanum Wheels](#understanding-mecanum-wheels)
- [Motor Direction Setup](#motor-direction-setup)
- [Tunable Parameters](#tunable-parameters)
- [Input Processing Pipeline](#input-processing-pipeline)
- [Tuning Procedures](#tuning-procedures)
- [Troubleshooting](#troubleshooting)
- [Advanced Tuning](#advanced-tuning)
- [Learning Resources](#learning-resources)

---

## Understanding Mecanum Wheels

### How Mecanum Wheels Work

Mecanum wheels have angled rollers (45°) that create force vectors when spinning. By combining 4 wheels with different roller orientations, the robot can move in any direction:

```
    FRONT OF ROBOT

   \\              //      ← Front wheels: rollers angle INWARD
   FL              FR


   //              \\      ← Back wheels: rollers angle INWARD
   BL              BR

    BACK OF ROBOT
```

**Key Principle**: When viewed from above, the rollers should form an **"X" pattern** pointing toward the center of the robot.

### Movement Combinations

| Movement | FL | FR | BL | BR | Result |
|----------|:--:|:--:|:--:|:--:|--------|
| Forward  | +  | +  | +  | +  | All wheels push forward |
| Backward | -  | -  | -  | -  | All wheels push backward |
| Strafe Right | + | - | - | + | Diagonal pairs work together |
| Strafe Left | - | + | + | - | Opposite diagonal pairs |
| Rotate CW | + | - | + | - | Left side forward, right backward |
| Rotate CCW | - | + | - | + | Right side forward, left backward |

---

## Motor Direction Setup

### Standard Configuration

In `PickleTeleOp.java`, motors are configured as:

```java
// Left side motors are REVERSED
frontLeft.setDirection(DcMotor.Direction.REVERSE);
backLeft.setDirection(DcMotor.Direction.REVERSE);

// Right side motors are FORWARD
frontRight.setDirection(DcMotor.Direction.FORWARD);
backRight.setDirection(DcMotor.Direction.FORWARD);
```

### Why Reverse Left Motors?

Motors on opposite sides of the robot face opposite directions. When both sides receive positive power:
- Left motors (reversed): Spin "backward" mechanically → wheels move forward
- Right motors (forward): Spin "forward" → wheels move forward
- **Result**: Robot drives straight forward

### Direction Test Procedure

1. **Lift the robot** so wheels are off the ground
2. **Push left stick forward** slightly
3. **Observe all 4 wheels**:
   - All should spin in the "forward" direction
   - Looking from behind, left wheels spin counterclockwise, right wheels spin clockwise

**If wheels spin incorrectly**, adjust the direction settings:

| Problem | Solution |
|---------|----------|
| Robot goes backward | Swap ALL directions (REVERSE↔FORWARD) |
| Robot spins in place | One side is wrong - check left vs right |
| One wheel is backwards | That motor's direction needs to be swapped |

---

## Tunable Parameters

### Location in Code

All tunable parameters are defined at the top of `PickleTeleOp.java`:

```java
// Speed control constants
final double NORMAL_DRIVE_SPEED = 0.7;   // 70% max speed
final double SLOW_DRIVE_SPEED = 0.3;     // 30% precision mode
final double STRAFE_MULTIPLIER = 1.1;    // Strafe compensation

// In mecanumDrive() function
final double DEADBAND = 0.05;            // 5% stick threshold
```

### Parameter Reference

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `NORMAL_DRIVE_SPEED` | 0.7 | 0.3 - 1.0 | Main driving speed multiplier |
| `SLOW_DRIVE_SPEED` | 0.3 | 0.1 - 0.5 | Precision mode speed (left bumper) |
| `STRAFE_MULTIPLIER` | 1.1 | 1.0 - 1.3 | Compensates for strafe inefficiency |
| `DEADBAND` | 0.05 | 0.02 - 0.1 | Joystick dead zone threshold |

---

## Input Processing Pipeline

Your joystick inputs go through multiple processing stages before reaching the motors:

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌─────────────┐
│ Raw Joystick│───▶│ Scaled       │───▶│ Input       │───▶│ Strafe      │
│ Input       │    │ Deadband     │    │ Shaping     │    │ Multiplier  │
└─────────────┘    └──────────────┘    └─────────────┘    └─────────────┘
                          │                   │                   │
                          ▼                   ▼                   ▼
                   Eliminates          Finer control       Compensates
                   stick drift         at low speeds       strafe loss
                          │                   │                   │
                          └───────────────────┼───────────────────┘
                                              ▼
                                   ┌─────────────────────┐
                                   │ Mecanum Drive Math  │
                                   │ FL = f + s + r      │
                                   │ FR = f - s - r      │
                                   │ BL = f - s + r      │
                                   │ BR = f + s - r      │
                                   └─────────────────────┘
                                              │
                                              ▼
                                   ┌─────────────────────┐
                                   │ Normalization       │
                                   │ (prevents clipping) │
                                   └─────────────────────┘
                                              │
                                              ▼
                                   ┌─────────────────────┐
                                   │ Speed Multiplier    │
                                   │ (normal/slow mode)  │
                                   └─────────────────────┘
                                              │
                                              ▼
                                        Motor Powers
```

### Stage 1: Scaled Deadband

Eliminates joystick drift while providing smooth response:

```
Input:  0.00  0.05  0.10  0.30  0.50  0.70  1.00
Output: 0.00  0.00  0.05  0.26  0.47  0.68  1.00
                ↑
          Threshold (no jump!)
```

### Stage 2: Input Shaping (Quadratic)

Squares the input for finer low-speed control:

```
Power
  1.0 │                           ___─── Linear
      │                      __───
  0.5 │                 __───
      │            __───      ___────── Quadratic (your code)
 0.25 │       __───      ___───
      │  __───      ___───
  0.0 └───────────────────────────────
      0        0.5        1.0
              Stick Position
```

### Stage 3: Strafe Multiplier

Boosts strafe power to compensate for roller friction:

```
Raw strafe input:     0.5
After multiplier:     0.5 × 1.1 = 0.55
```

---

## Tuning Procedures

### Step 1: Basic Direction Test

1. Power on robot with wheels lifted off ground
2. Push left stick forward → all wheels forward
3. Push left stick right → robot should strafe right
4. Push right stick right → robot should rotate clockwise

**Fix any direction issues before proceeding!**

### Step 2: Strafe Multiplier Tuning

**Goal**: Make strafing feel as responsive as forward/backward movement.

1. Set `STRAFE_MULTIPLIER = 1.0` (neutral)
2. Drive forward at 50% stick → note the speed
3. Strafe right at 50% stick → compare the speed
4. If strafing is slower, increase multiplier:

```java
// Try these values:
final double STRAFE_MULTIPLIER = 1.1;  // Start here
final double STRAFE_MULTIPLIER = 1.15; // If still slow
final double STRAFE_MULTIPLIER = 1.2;  // For heavy robots
```

**Warning**: Don't exceed 1.3 - it can cause normalization issues.

### Step 3: Speed Multiplier Tuning

**Goal**: Find the right balance between speed and control.

| Driver Experience | `NORMAL_DRIVE_SPEED` | `SLOW_DRIVE_SPEED` |
|-------------------|----------------------|--------------------|
| Beginner | 0.5 - 0.6 | 0.2 - 0.3 |
| Intermediate | 0.6 - 0.7 | 0.3 - 0.4 |
| Advanced | 0.7 - 0.85 | 0.35 - 0.5 |
| Expert | 0.85 - 1.0 | 0.4 - 0.6 |

**Tuning Process**:
1. Start at 0.5 for beginners
2. Practice driving for 5 minutes
3. If robot feels slow, increase by 0.1
4. If robot is hard to control, decrease by 0.1

### Step 4: Deadband Tuning

**Goal**: Eliminate drift without losing responsiveness.

1. Place robot on the field, hands off controller
2. Watch if robot drifts without input
3. Adjust deadband:

```java
// If robot drifts at rest:
final double DEADBAND = 0.08;  // Increase threshold

// If controls feel unresponsive:
final double DEADBAND = 0.03;  // Decrease threshold
```

---

## Troubleshooting

### Problem: Robot Drifts When Strafing

**Symptoms**: Robot moves forward/backward while trying to strafe sideways.

**Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| Mecanum wheels mounted wrong | Check "X" pattern from above |
| Motors not calibrated equally | Use `RUN_USING_ENCODER` mode |
| Weight distribution uneven | Balance the robot's center of gravity |
| Floor surface inconsistent | Practice on competition-similar surface |

### Problem: Robot Spins Instead of Driving Straight

**Symptoms**: Pushing forward makes robot rotate instead of drive.

**Solutions**:
1. Check motor directions - left and right sides may be swapped
2. Verify hardware configuration names match code
3. Check if one motor is unplugged or damaged

### Problem: Strafing Goes Wrong Direction

**Symptoms**: Pushing stick right makes robot strafe left.

**Solutions**:
1. Mecanum wheels may be in "O" pattern instead of "X"
2. Swap front-left with back-right, AND front-right with back-left
3. Or negate the strafe input in code:
   ```java
   mecanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
   //                                   ↑ Added negative sign
   ```

### Problem: Robot Feels "Sluggish" or "Laggy"

**Symptoms**: Robot responds slowly to joystick input.

**Solutions**:
1. Check battery voltage (should be > 12V)
2. Reduce input shaping strength (see Advanced Tuning)
3. Increase speed multipliers
4. Consider switching to `RUN_WITHOUT_ENCODER` for faster response

### Problem: One Corner is Weaker

**Symptoms**: Robot pulls to one side or one wheel seems slower.

**Solutions**:
1. Check motor connections and wiring
2. Swap suspect motor with known-good motor to isolate issue
3. Check for mechanical binding or friction
4. Verify encoder is working if using `RUN_USING_ENCODER`

---

## Advanced Tuning

### Alternative Input Shaping Curves

The default quadratic curve may not suit all drivers. Here are alternatives:

#### Linear (No Shaping)

Direct 1:1 response - maximum responsiveness, less precision:

```java
double shapeInput(double value) {
    return value;  // No modification
}
```

#### Cubic (More Precision)

Even more sensitive at low speeds than quadratic:

```java
double shapeInput(double value) {
    return value * value * value;  // Cubic curve
}
```

#### Custom Curve

Blend between linear and quadratic:

```java
double shapeInput(double value) {
    double blend = 0.5;  // 0.0 = quadratic, 1.0 = linear
    double quadratic = Math.copySign(value * value, value);
    return blend * value + (1 - blend) * quadratic;
}
```

### Field-Centric Drive

For advanced teams, consider implementing field-centric drive using the IMU:

```java
// Concept - requires IMU integration
double heading = imu.getAngularOrientation().firstAngle;
double rotatedX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
double rotatedY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
mecanumDrive(rotatedY, rotatedX, rotate);
```

This makes "forward" always mean "toward the field's far end" regardless of robot orientation.

### Telemetry for Debugging

Add detailed telemetry to diagnose issues:

```java
// In loop():
telemetry.addData("Raw Inputs", "F:%.2f S:%.2f R:%.2f",
    -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
telemetry.addData("Motor Powers", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
telemetry.addData("Encoders", "FL:%d FR:%d BL:%d BR:%d",
    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
```

---

## Learning Resources

### Official Documentation

- [FTC Docs - Mecanum Drive](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/index.html) - Official FTC programming tutorials
- [gm0 - Mecanum Drivetrains](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html) - Game Manual 0's comprehensive guide
- [REV Robotics - Mecanum Wheels](https://docs.revrobotics.com/duo-build/actuators/wheels) - Hardware specifications

### Video Tutorials

- [FTC Programming Tutorial - Mecanum Drive](https://www.youtube.com/watch?v=gnSW2QpkGXQ)
- [goBILDA Mecanum Tutorial](https://www.youtube.com/watch?v=v_iYlpNuJ8E)

### Community Resources

- [FTC Discord](https://discord.gg/ftc) - Real-time help from other teams
- [FTC Reddit](https://reddit.com/r/FTC) - Discussion and Q&A
- [Chief Delphi](https://www.chiefdelphi.com/c/first-tech-challenge/41) - Forums for FTC and FRC

### Sample Code

- [FtcRobotController Samples](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples) - Official sample OpModes
- [Road Runner](https://learnroadrunner.com/) - Advanced autonomous path planning (uses mecanum math)

---

## Quick Reference Card

Print this for the pit area:

```
┌─────────────────────────────────────────────────────────────┐
│                 MECANUM TUNING QUICK REFERENCE              │
├─────────────────────────────────────────────────────────────┤
│ MOTOR DIRECTIONS                                            │
│   Left motors:  REVERSE     Right motors: FORWARD           │
│                                                             │
│ WHEEL PATTERN (view from above)                             │
│       \\  //     Rollers form "X" pointing inward           │
│       //  \\                                                │
│                                                             │
│ CONTROLS                                                    │
│   Left stick Y:  Forward/Backward                           │
│   Left stick X:  Strafe Left/Right                          │
│   Right stick X: Rotate                                     │
│   Left bumper:   Toggle slow mode                           │
│                                                             │
│ TUNABLE VALUES                                              │
│   NORMAL_DRIVE_SPEED = 0.7    (range: 0.3 - 1.0)            │
│   SLOW_DRIVE_SPEED = 0.3      (range: 0.1 - 0.5)            │
│   STRAFE_MULTIPLIER = 1.1     (range: 1.0 - 1.3)            │
│   DEADBAND = 0.05             (range: 0.02 - 0.1)           │
│                                                             │
│ TROUBLESHOOTING                                             │
│   Drifts when idle → Increase DEADBAND                      │
│   Strafe feels slow → Increase STRAFE_MULTIPLIER            │
│   Too fast/twitchy → Decrease NORMAL_DRIVE_SPEED            │
│   Wrong direction → Check motor direction config            │
└─────────────────────────────────────────────────────────────┘
```

---

*Last updated: December 2024*
*For: Pickle Robot - FTC DECODE Season 2025-2026*
