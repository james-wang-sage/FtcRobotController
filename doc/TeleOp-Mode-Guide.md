# DECODE TeleOp Mode Guide

> **Complete driver guide** for PickleTeleOp with mecanum drive, launcher controls, and the auto-align feature for optimal ball launching.

## Table of Contents

1. [Overview](#overview)
2. [Control Scheme](#control-scheme)
3. [Auto-Align Feature (L1)](#auto-align-feature-l1)
4. [Drive Modes](#drive-modes)
5. [Launcher System](#launcher-system)
6. [Pre-Match Setup](#pre-match-setup)
7. [Telemetry Guide](#telemetry-guide)
8. [Tuning Parameters](#tuning-parameters)
9. [Troubleshooting](#troubleshooting)
10. [Learning Path](#learning-path)

---

## Overview

The **PickleTeleOp** OpMode provides driver-controlled operation of the Pickle robot for the DECODE 2025-2026 season. It features:

- **4-motor mecanum drive** for omnidirectional movement
- **High-speed ball launcher** with velocity-controlled motor
- **Dual feeder servos** for consistent ball feeding
- **IMU-based auto-align** for precise goal targeting
- **Slow mode** for precision driving

### Robot Capabilities

```
┌─────────────────────────────────────────────────────────────────┐
│                      PICKLE ROBOT OVERVIEW                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   DRIVETRAIN: 4-wheel mecanum (omnidirectional)                 │
│   ─────────────────────────────────────────────                 │
│       ┌─────┐            ┌─────┐                                │
│       │ FL  │            │ FR  │   • Full X/Y/rotation control  │
│       └─────┘            └─────┘   • Encoder-based velocity     │
│                                    • Brake mode for quick stops │
│       ┌─────┐            ┌─────┐                                │
│       │ BL  │            │ BR  │                                │
│       └─────┘            └─────┘                                │
│                                                                 │
│   LAUNCHER: High-speed flywheel system                          │
│   ────────────────────────────────────                          │
│       • Velocity-controlled motor (1125 ticks/sec target)       │
│       • Dual feeder servos for ball intake                      │
│       • 2-second cooldown between shots                         │
│                                                                 │
│   SENSORS: IMU for heading                                      │
│   ────────────────────────                                      │
│       • Auto-align to goal perpendicular heading                │
│       • Real-time heading display                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Control Scheme

### Primary Controls (Gamepad 1)

```
┌─────────────────────────────────────────────────────────────────┐
│                    GAMEPAD 1 - DRIVER CONTROLS                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                 ┌─────────────────────────┐                     │
│                 │    [LB]         [RB]    │                     │
│                 │  Slow Mode    LAUNCH    │                     │
│                 │    Toggle      Shot     │                     │
│                 │                         │                     │
│                 │   [LT]         [RT]     │                     │
│                 │  AUTO-ALIGN   (unused)  │                     │
│                 │   to Goal               │                     │
│                 │                         │                     │
│          ┌──────┴──────┐    ┌──────┴──────┐                     │
│          │    [Y]      │    │     [B]     │                     │
│          │  Spin Up    │    │  Stop       │                     │
│          │  Launcher   │    │  Launcher   │                     │
│          │             │    │             │                     │
│    ┌─────┤  [X]   [A]  ├────┤             │                     │
│    │     │(unused)     │    │             │                     │
│    │     └─────────────┘    └─────────────┘                     │
│    │                                                            │
│    │     ┌─────┐              ┌─────┐                           │
│    │     │     │              │     │                           │
│    └────▶│  L  │              │  R  │◀─── Rotation              │
│          │     │              │     │                           │
│          └──┬──┘              └──┬──┘                           │
│   Forward/  │                    │                              │
│   Strafe    │                    │                              │
│             ▼                    ▼                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Control Summary Table

| Control | Action | Notes |
|---------|--------|-------|
| **Left Stick Y** | Forward / Backward | Push forward to drive forward |
| **Left Stick X** | Strafe Left / Right | Push right to strafe right |
| **Right Stick X** | Rotate | Push right to turn clockwise |
| **Left Bumper (LB)** | Toggle Slow Mode | 30% speed for precision |
| **Right Bumper (RB)** | Launch Ball | Triggers full launch sequence |
| **Left Trigger (LT)** | Auto-Align to Goal | Hold to align perpendicular to goal |
| **Y Button** | Spin Up Launcher | Manually spin up flywheel |
| **B Button** | Stop Launcher | Stop flywheel motor |
| **X Button** | Select BLUE Alliance | During init only |
| **B Button** | Select RED Alliance | During init only |

### Joystick Orientation

```
LEFT STICK:                          RIGHT STICK:
─────────────                        ─────────────

      ↑ Forward
      │
   ←──┼──→ Strafe                    ←──┼──→ Rotate
      │
      ↓ Backward

  Push UP = robot moves forward      Push RIGHT = robot turns clockwise
  Push RIGHT = robot strafes right   Push LEFT = robot turns counter-clockwise
```

---

## Auto-Align Feature (L1)

The **Auto-Align** feature automatically rotates the robot to face perpendicular to the goal zone border, providing the optimal angle for ball launching.

### How It Works

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTO-ALIGN MECHANISM                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   DECODE Field Goal Zones (top-down view):                      │
│                                                                 │
│          BLUE GOAL                      RED GOAL                │
│            ╲                              ╱                     │
│             ╲ 135°                  45° ╱                       │
│              ╲                        ╱                         │
│               ╲                      ╱                          │
│                ╲                    ╱                           │
│   For BLUE:     ╲                  ╱     For RED:               │
│   Target = 45°   ●────────────────●     Target = 135°           │
│   (face NE)                             (face NW)               │
│                                                                 │
│   The robot rotates to face PERPENDICULAR to the angled        │
│   goal border, giving optimal ball trajectory into the goal.   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Target Headings

| Alliance | Target Heading | Direction |
|----------|----------------|-----------|
| **RED** | 135° | Northwest (toward red goal opening) |
| **BLUE** | 45° | Northeast (toward blue goal opening) |

### Usage Steps

1. **Select Alliance** (during init): Press **X** for Blue or **B** for Red
2. **Drive near goal zone**: Position robot in launching range
3. **Hold L1 (Left Trigger)**: Robot automatically rotates to target heading
4. **Watch telemetry**: Shows current heading → target heading (error)
5. **When aligned**: "Align State: ALIGNED" appears
6. **Launch**: Press **RB** to fire while holding L1

### State Machine

```
┌──────────────────────────────────────────────────────────────┐
│                   AUTO-ALIGN STATE MACHINE                    │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌──────────┐                                               │
│   │   IDLE   │◀─────────────────────────────────────┐        │
│   │          │        L1 Released                   │        │
│   └────┬─────┘                                      │        │
│        │ L1 Pressed                                 │        │
│        ▼                                            │        │
│   ┌──────────┐                                      │        │
│   │ ALIGNING │  Robot rotating toward target        │        │
│   │          │  heading using P-controller          │        │
│   └────┬─────┘                                      │        │
│        │ Within ±3° tolerance                       │        │
│        ▼                                            │        │
│   ┌──────────┐                                      │        │
│   │ ALIGNED  │  Ready to launch!                    │        │
│   │          │  (still holding L1)                  │        │
│   └────┬─────┘                                      │        │
│        │ L1 Released                                │        │
│        └────────────────────────────────────────────┘        │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### Driver Tips for Auto-Align

1. **Position first, then align**: Get close to launching position before pressing L1
2. **You can still drive**: Forward/backward and strafe work during alignment
3. **Release to cancel**: Let go of L1 to return to manual rotation control
4. **Check telemetry**: The heading error shows how far you need to rotate
5. **Small adjustments**: If slightly off after aligning, tap L1 again

---

## Drive Modes

### Normal Mode (Default)

- **Speed**: 70% of maximum motor power
- **Best for**: General driving, approaching positions quickly

### Slow Mode (Left Bumper Toggle)

- **Speed**: 30% of maximum motor power
- **Best for**: Precision alignment, delicate maneuvers near obstacles
- **Toggle**: Press LB once to enable, press again to disable

```
┌─────────────────────────────────────────────────────────────────┐
│                      DRIVE MODES COMPARISON                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   NORMAL MODE (70%)              SLOW MODE (30%)                │
│   ─────────────────              ─────────────────              │
│                                                                 │
│   ████████████████░░░░░░        ██████░░░░░░░░░░░░░░            │
│   └────────┬───────┘            └──┬──┘                         │
│            │                       │                            │
│   • Fast traversal               • Precise positioning          │
│   • Aggressive driving           • Lining up shots              │
│   • Covering distance            • Avoiding collisions          │
│   • Default state                • Toggle with LB               │
│                                                                 │
│   Telemetry shows: "NORMAL (70%)" or "SLOW (30%)"               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Input Processing

The TeleOp includes several input processing features:

1. **Deadband (5%)**: Eliminates joystick drift - small movements ignored
2. **Quadratic Scaling**: Squares input for fine low-speed control
3. **Strafe Compensation (1.1x)**: Compensates for mecanum strafe inefficiency
4. **Power Normalization**: Prevents any motor from exceeding ±1.0

```
Input Processing Pipeline:
──────────────────────────

Raw Joystick ──▶ Deadband ──▶ Quadratic ──▶ Strafe ──▶ Mecanum ──▶ Normalize ──▶ Motors
   Input         Filter       Scaling       Mult       Math        Powers
```

---

## Launcher System

### Launch Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                      LAUNCH STATE MACHINE                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌────────┐      RB Pressed       ┌──────────┐                 │
│   │  IDLE  │──────────────────────▶│ SPIN_UP  │                 │
│   │        │◀───────────┐          │          │                 │
│   └────────┘            │          └────┬─────┘                 │
│                         │               │ Velocity > 1075       │
│      Cooldown           │               ▼                       │
│      Complete           │          ┌──────────┐                 │
│         (2s)            │          │  LAUNCH  │ Feeders ON      │
│                         │          │          │                 │
│   ┌────────────┐        │          └────┬─────┘                 │
│   │ LAUNCHING  │────────┘               │                       │
│   │            │◀───────────────────────┘                       │
│   └────────────┘        Feed Time: 0.2s                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Launcher Controls

| Control | Action |
|---------|--------|
| **RB** | Full launch sequence (spin up → feed → cooldown) |
| **Y** | Manually spin up flywheel (for pre-heating) |
| **B** | Stop flywheel motor |

### Launch Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Target Velocity | 1125 ticks/sec | Flywheel target speed |
| Min Velocity | 1075 ticks/sec | Minimum before feeding |
| Feed Time | 0.2 seconds | How long feeders run |
| Cooldown | 2.0 seconds | Time between shots |

### Pro Tips for Launching

1. **Pre-spin**: Hold **Y** to spin up before reaching launch position
2. **Wait for velocity**: Watch telemetry - launch when "Launcher Speed" is stable
3. **Cooldown timer**: Check "Launch Cooldown" or "READY" in telemetry
4. **Consistent distance**: Same distance = more consistent shots

---

## Pre-Match Setup

### Init Phase Checklist

During the init phase (after pressing INIT but before START):

```
┌─────────────────────────────────────────────────────────────────┐
│                    INIT PHASE CHECKLIST                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   [ ] 1. SELECT ALLIANCE                                        │
│         • Press X for BLUE alliance                             │
│         • Press B for RED alliance                              │
│         • Verify "Current Alliance: RED/BLUE" in telemetry      │
│                                                                 │
│   [ ] 2. VERIFY IMU                                             │
│         • Check "IMU: Ready" appears                            │
│         • Current heading should update when robot rotates      │
│                                                                 │
│   [ ] 3. CHECK TARGET HEADING                                   │
│         • RED = 135° (northwest)                                │
│         • BLUE = 45° (northeast)                                │
│                                                                 │
│   [ ] 4. POSITION ROBOT                                         │
│         • Place on starting tile                                │
│         • Note: Current heading shown in telemetry              │
│                                                                 │
│   [ ] 5. PRESS START                                            │
│         • Alliance selection is locked after START              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Init Telemetry Display

```
--- ALLIANCE SELECTION ---
Press X: for BLUE
Press B: for RED
Current Alliance: RED

Target Heading: 135.0°
Current IMU Heading: 87.3°
```

---

## Telemetry Guide

### Main TeleOp Telemetry

```
┌─────────────────────────────────────────────────────────────────┐
│                    TELEOP TELEMETRY DISPLAY                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Alliance: RED                    ◀── Your selected alliance   │
│   Drive Mode: NORMAL (70%)         ◀── Current speed mode       │
│   Launch State: IDLE               ◀── Launcher status          │
│                                                                 │
│   Align State: ALIGNING            ◀── Auto-align status        │
│   Heading: 87.3° → 135.0° (err: 47.7°)                          │
│            └──┬──┘  └──┬──┘  └───┬───┘                          │
│           Current   Target    How far                           │
│                                                                 │
│   Launch Cooldown: 1.2 sec         ◀── Or "Launch Status: READY"│
│                                                                 │
│   Front Motors: L:0.45  R:0.45     ◀── Motor power levels       │
│   Back Motors:  L:0.45  R:0.45                                  │
│   Launcher Speed: 1125             ◀── Current flywheel velocity│
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Understanding Telemetry Values

| Telemetry | Meaning | What to Look For |
|-----------|---------|------------------|
| Alliance | Selected team | Should match your field position |
| Drive Mode | Speed setting | SLOW during alignment |
| Launch State | Launcher progress | IDLE when ready |
| Align State | Auto-align status | ALIGNED when ready to fire |
| Heading | Current → Target (error) | Error approaching 0° |
| Launch Cooldown | Time until ready | Shows "READY" when can fire |
| Launcher Speed | Flywheel velocity | ~1125 when at speed |

---

## Tuning Parameters

### Drive Parameters

Located at the top of `PickleTeleOp.java`:

```java
// Speed control constants
final double NORMAL_DRIVE_SPEED = 0.7;  // 70% - adjust for faster/slower driving
final double SLOW_DRIVE_SPEED = 0.3;    // 30% - adjust for precision mode

// Strafe compensation
final double STRAFE_MULTIPLIER = 1.1;   // Increase if strafing feels weak
```

### Auto-Align Parameters

```java
// Target headings (in degrees)
final double RED_GOAL_PERPENDICULAR_HEADING_DEG = 135.0;   // Adjust if goal angle differs
final double BLUE_GOAL_PERPENDICULAR_HEADING_DEG = 45.0;

// Alignment behavior
final double ALIGN_TOLERANCE_DEG = 3.0;      // How close is "aligned" (smaller = more precise)
final double ALIGN_ROTATION_SPEED = 0.35;    // Max rotation speed during align
final double ALIGN_HEADING_KP = 0.015;       // P-controller gain (higher = faster but may overshoot)
```

### Launcher Parameters

```java
// Velocity control
final double LAUNCHER_TARGET_VELOCITY = 1125;  // Target flywheel speed
final double LAUNCHER_MIN_VELOCITY = 1075;     // Min speed before feeding

// Timing
final double FEED_TIME_SECONDS = 0.20;         // How long feeders run
final double LAUNCH_COOLDOWN_SECONDS = 2.0;    // Time between shots
```

### IMU Orientation

If the robot rotates the wrong direction during auto-align:

```java
// In init() method - adjust to match your hub mounting
RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,        // Change if hub mounted differently
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD     // Change if USB faces different direction
);
```

---

## Troubleshooting

### Problem: Robot doesn't align to correct heading

**Possible causes:**
1. Wrong alliance selected
2. IMU orientation incorrect
3. Target heading values wrong for your field setup

**Solutions:**
1. Check telemetry shows correct alliance (RED/BLUE)
2. Verify IMU orientation in code matches physical hub mounting
3. Adjust `RED_GOAL_PERPENDICULAR_HEADING_DEG` and `BLUE_GOAL_PERPENDICULAR_HEADING_DEG`

---

### Problem: Auto-align rotates wrong direction (takes long path)

**Cause:** Angle normalization issue or IMU direction reversed

**Solution:** Check USB facing direction in IMU initialization:
```java
// Try different USB directions:
RevHubOrientationOnRobot.UsbFacingDirection.FORWARD   // Try these
RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD  // options
RevHubOrientationOnRobot.UsbFacingDirection.LEFT
RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
```

---

### Problem: Robot overshoots target heading during align

**Cause:** P-gain too high or rotation speed too fast

**Solution:** Reduce tuning parameters:
```java
final double ALIGN_ROTATION_SPEED = 0.25;  // Lower max speed
final double ALIGN_HEADING_KP = 0.010;     // Lower P-gain
```

---

### Problem: Robot can't reach target heading (oscillates)

**Cause:** Tolerance too tight or floor friction high

**Solution:** Increase tolerance:
```java
final double ALIGN_TOLERANCE_DEG = 5.0;  // More forgiving
```

---

### Problem: Launcher not reaching target velocity

**Possible causes:**
1. Motor not connected properly
2. PIDF coefficients need tuning
3. Mechanical resistance

**Debug:**
1. Check "Launcher Speed" in telemetry - should increase when Y pressed
2. Listen for motor spinning up
3. Check for binding in launcher mechanism

---

### Problem: Drive feels sluggish or unresponsive

**Solutions:**
1. Increase speed multiplier:
   ```java
   final double NORMAL_DRIVE_SPEED = 0.85;  // Higher speed
   ```
2. Check motor mode is `RUN_USING_ENCODER` (not `RUN_TO_POSITION`)
3. Verify all 4 drive motors are connected

---

### Problem: IMU shows "NOT AVAILABLE"

**Cause:** IMU not found in hardware config or initialization failed

**Solutions:**
1. Check Driver Station hardware config includes "imu"
2. Verify Control Hub is properly connected
3. Check for error messages in telemetry

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────┐
│                  PICKLE TELEOP QUICK REFERENCE                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  INIT PHASE:                                                    │
│    • Press X = BLUE alliance                                    │
│    • Press B = RED alliance                                     │
│    • Verify IMU status and heading                              │
│                                                                 │
│  DRIVING:                                                       │
│    Left Stick Y  = Forward/Backward                             │
│    Left Stick X  = Strafe Left/Right                            │
│    Right Stick X = Rotate                                       │
│    LB            = Toggle Slow Mode (30%)                       │
│                                                                 │
│  AUTO-ALIGN:                                                    │
│    Hold LT       = Auto-rotate to goal-perpendicular heading    │
│    Release LT    = Return to manual rotation                    │
│    Target:  RED = 135° (NW)  |  BLUE = 45° (NE)                 │
│                                                                 │
│  LAUNCHING:                                                     │
│    Y             = Spin up launcher (manual)                    │
│    B             = Stop launcher                                │
│    RB            = Full launch sequence                         │
│    Cooldown: 2 seconds between shots                            │
│                                                                 │
│  TELEMETRY KEY VALUES:                                          │
│    Align State: IDLE → ALIGNING → ALIGNED                       │
│    Heading: current° → target° (error°)                         │
│    Launch Status: READY (or cooldown timer)                     │
│    Launcher Speed: ~1125 when ready                             │
│                                                                 │
│  DRIVER WORKFLOW:                                               │
│    1. Drive toward goal zone                                    │
│    2. Hold LT to auto-align                                     │
│    3. Wait for "ALIGNED" state                                  │
│    4. Press RB to launch                                        │
│    5. Release LT, reposition, repeat                            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Learning Path

### Beginner

1. **Understand mecanum drive**
   - [gm0: Mecanum Drives](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)
   - [Mecanum Drive Guide](Mecanum-Drive-Guide.md) (local)

2. **Learn about the IMU**
   - [FTC IMU Guide](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)
   - [IMU TeleOp Guide](IMU-TeleOp-Guide.md) (local)

3. **Understand state machines**
   - [FTC State Machines](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/state_machine/state-machine.html)

### Intermediate

4. **Learn proportional control**
   - [CTRL ALT FTC: PID Introduction](https://www.ctrlaltftc.com/the-pid-controller)

5. **Input shaping and filtering**
   - [gm0: Controls](https://gm0.org/en/latest/docs/software/concepts/control-loops.html)

6. **Velocity control for motors**
   - [REV Motor Control](https://docs.revrobotics.com/duo-control/programming/using-motor-encoders)

### Advanced

7. **Field coordinate systems**
   - [FTC Coordinate System](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)

8. **Advanced drive techniques**
   - [Advanced Patterns from BunyipsFTC](Advanced-Patterns-From-BunyipsFTC.md) (local)

9. **Reference implementations**
   - [FTCLib: Gamepad Features](https://docs.ftclib.org/ftclib/features/gamepad-extensions)

---

## File Reference

| File | Location | Purpose |
|------|----------|---------|
| `PickleTeleOp.java` | `pickle/` | Main TeleOp OpMode |
| `PickleHardwareNames.java` | `pickle/config/` | Hardware configuration names |
| `Alliance.java` | `pickle/field/` | Alliance enum (RED/BLUE) |
| `DecodeField.java` | `pickle/field/` | Field positions and constants |
| `MecanumDriveHelper.java` | `pickle/drive/` | Drive control helper (used in Auto) |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Dec 2024 | Initial release with mecanum drive and launcher |
| 1.1 | Dec 2024 | Added L1 auto-align to goal feature |
