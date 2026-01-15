# DECODE Holonomic Autonomous Guide

> **Advanced autonomous mode** using sensor fusion, encoder odometry, AprilTag localization, and Road Runner-style path following.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Sensor Fusion](#sensor-fusion)
4. [Components](#components)
   - [MecanumOdometry](#mecanumodometry)
   - [AprilTagLocalizer](#apriltaglocalizer)
   - [PathFollower](#pathfollower)
5. [Tuning Guide](#tuning-guide)
6. [Path Building API](#path-building-api)
7. [Troubleshooting](#troubleshooting)
8. [Learning Path](#learning-path)

---

## Overview

The **PickleAutoHolonomic** OpMode represents an advanced autonomous implementation that combines multiple positioning systems for accurate robot navigation.

### Comparison: Basic vs Holonomic

| Feature | PickleAutoOp (Basic) | PickleAutoHolonomic (Advanced) |
|---------|---------------------|-------------------------------|
| Position Tracking | IMU heading only | Full X/Y/θ odometry |
| AprilTag Use | Goal identification | Position correction |
| Path Following | Sequential moves | Simultaneous X/Y/rotation |
| Position Updates | ~10 Hz (visual) | ~50 Hz (encoders) |
| Drift Correction | None | Continuous fusion |

### Path Comparison

```
Basic (Sequential):              Holonomic (Simultaneous):
┌─────────────────────┐          ┌─────────────────────┐
│         GOAL        │          │         GOAL        │
│          ▲          │          │          ▲          │
│          │ step 3   │          │         /           │
│    ┌─────┘          │          │        / diagonal   │
│    │ step 2         │          │       /  + rotate   │
│    │                │          │      /              │
│    └─ step 1        │          │     /               │
│    ●                │          │    ●                │
│  START              │          │  START              │
└─────────────────────┘          └─────────────────────┘
  ~4-5 seconds                     ~2-3 seconds
```

---

## Architecture

### System Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PickleAutoHolonomic                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  SENSORS                    PROCESSING                    ACTUATORS     │
│  ────────                   ──────────                    ─────────     │
│                                                                         │
│  ┌──────────┐              ┌──────────────┐                             │
│  │  Wheel   │─────────────▶│  Mecanum     │                             │
│  │ Encoders │  tick deltas │  Odometry    │                             │
│  │  (x4)    │              └──────┬───────┘                             │
│  └──────────┘                     │                                     │
│                                   │ pose estimate                       │
│  ┌──────────┐              ┌──────▼───────┐              ┌───────────┐  │
│  │   IMU    │─────────────▶│    Sensor    │─────────────▶│   Path    │  │
│  │ (heading)│   heading    │    Fusion    │  fused pose  │  Follower │  │
│  └──────────┘              └──────▲───────┘              └─────┬─────┘  │
│                                   │                            │        │
│  ┌──────────┐              ┌──────┴───────┐              ┌─────▼─────┐  │
│  │ AprilTag │─────────────▶│  AprilTag    │              │  Mecanum  │  │
│  │  Camera  │  detections  │  Localizer   │              │   Drive   │  │
│  └──────────┘              └──────────────┘              │  Helper   │  │
│                                                          └─────┬─────┘  │
│                                                                │        │
│                                                          ┌─────▼─────┐  │
│                                                          │  Motors   │  │
│                                                          │  (x4)     │  │
│                                                          └───────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

![System Block Diagram](system.block.diagram.png)

### State Machine

```
┌─────────────────────────────────────────────────────────────────┐
│                        AUTO STATE MACHINE                       │
└─────────────────────────────────────────────────────────────────┘

  ┌───────────────────┐
  │ FOLLOW_PATH_TO    │◀──── start()
  │    SHOOTING       │
  └─────────┬─────────┘
            │ path complete
            ▼
  ┌───────────────────┐
  │ SETTLE_AT         │ Fine-tune alignment
  │    POSITION       │ (0.3 seconds)
  └─────────┬─────────┘
            │
            ▼
  ┌───────────────────┐
  │ REQUEST_SHOT      │◀───────────┐
  └─────────┬─────────┘            │
            │                      │ shotsToFire > 0
            ▼                      │
  ┌───────────────────┐            │
  │ WAIT_FOR_SHOT     │────────────┘
  └─────────┬─────────┘
            │ shotsToFire == 0
            ▼
  ┌───────────────────┐
  │ FOLLOW_PATH_TO    │
  │       PARK        │
  └─────────┬─────────┘
            │ path complete
            ▼
  ┌───────────────────┐
  │     COMPLETE      │
  └───────────────────┘
```

---

## Sensor Fusion

### What is Sensor Fusion?

Sensor fusion combines data from multiple sensors to get a more accurate estimate than any single sensor could provide alone.

```
┌─────────────────────────────────────────────────────────────────┐
│                    WHY SENSOR FUSION?                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ENCODERS (Odometry)          APRILTAG (Vision)                 │
│  ──────────────────           ─────────────────                 │
│  ✓ High frequency (50Hz)      ✓ Absolute position               │
│  ✓ Works everywhere           ✓ No drift over time              │
│  ✗ Drifts over time           ✗ Low frequency (~10Hz)           │
│  ✗ Wheel slip errors          ✗ Needs line of sight             │
│                                                                 │
│                        ▼                                        │
│              ┌─────────────────┐                                │
│              │  SENSOR FUSION  │                                │
│              │  Best of both!  │                                │
│              └─────────────────┘                                │
│                                                                 │
│  ✓ High frequency updates                                       │
│  ✓ Absolute position reference                                  │
│  ✓ Drift correction                                             │
│  ✓ Works even when tags not visible                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Blend Factor Explained

The blend factor controls how much to trust AprilTag readings vs odometry:

```java
// From PickleAutoHolonomic.java:103
final double APRILTAG_BLEND_FACTOR = 0.4;  // 40% AprilTag, 60% odometry
```

**How it works:**

```
New Position = Odometry Position + (AprilTag Position - Odometry Position) × Blend Factor

Example:
  Odometry says: (1000, 500)
  AprilTag says: (1020, 510)
  Blend = 0.4

  New X = 1000 + (1020 - 1000) × 0.4 = 1000 + 8 = 1008
  New Y = 500 + (510 - 500) × 0.4 = 500 + 4 = 504

  Result: (1008, 504) - moved slightly toward AprilTag reading
```

**Choosing a blend factor:**

| Blend Factor | Behavior | When to Use |
|--------------|----------|-------------|
| 0.0 - 0.2 | Trust odometry almost entirely | Odometry is very accurate, tags unreliable |
| 0.3 - 0.5 | Balanced fusion | **Default - good starting point** |
| 0.6 - 0.8 | Trust AprilTag more | Large odometry drift, good tag visibility |
| 0.9 - 1.0 | Jump to AprilTag position | Only for debugging |

---

## Components

### MecanumOdometry

**File:** `pickle/odometry/MecanumOdometry.java`

Tracks robot position by integrating wheel encoder changes.

#### How Mecanum Odometry Works

```
Each wheel contributes to robot motion differently:

         FRONT
    ┌─────────────┐
    │ FL ╲   ╱ FR │    FL (Front Left):  +forward, +left, +rotate
    │     ╲ ╱     │    FR (Front Right): +forward, -left, -rotate
    │      ●      │    BL (Back Left):   +forward, -left, +rotate
    │     ╱ ╲     │    BR (Back Right):  +forward, +left, -rotate
    │ BL ╱   ╲ BR │
    └─────────────┘
         BACK

Forward Kinematics (encoder ticks → robot motion):
  ΔX (forward) = (FL + FR + BL + BR) / 4
  ΔY (strafe)  = (-FL + FR + BL - BR) / 4
  Δθ (rotate)  = (-FL + FR - BL + BR) / (4 × trackWidth)
```

#### Key Methods

```java
// Initialize with your robot's measurements
odometry.setWheelParameters(
    48.0,    // wheel radius in mm (GoBILDA mecanum)
    537.7,   // encoder ticks per revolution
    350.0,   // track width (left-right distance)
    300.0    // wheel base (front-back distance)
);

// In init()
odometry.resetPose(startingPose);

// In loop() - call EVERY iteration
odometry.update();
Pose2d currentPose = odometry.getPose();

// When AprilTag gives a correction
odometry.correctPose(visionPose, 0.4);
```

#### Tuning Wheel Parameters

**Measure these on YOUR robot:**

1. **Wheel Radius**: Measure wheel diameter, divide by 2
2. **Ticks Per Revolution**: Check motor spec sheet or run test
3. **Track Width**: Distance between left and right wheel centers
4. **Wheel Base**: Distance between front and back wheel centers

```
        ◀──── Track Width ────▶

     ┌─────┐            ┌─────┐   ▲
     │ FL  │            │ FR  │   │
     └──●──┘            └──●──┘   │
                                  │ Wheel
                                  │ Base
     ┌──●──┐            ┌──●──┐   │
     │ BL  │            │ BR  │   │
     └─────┘            └─────┘   ▼
```

---

### AprilTagLocalizer

**File:** `pickle/vision/AprilTagLocalizer.java`

Detects AprilTags and calculates robot position on the field.

#### DECODE Field Tags

```
┌─────────────────────────────────────────────────────────────────┐
│                         DECODE FIELD                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ┌─────┐                                         ┌─────┐      │
│    │ 24  │  RED GOAL                  BLUE GOAL    │ 20  │      │
│    │     │  (Use for                  (Use for     │     │      │
│    └─────┘   localization)            localization)└─────┘      │
│                                                                 │
│                                                                 │
│                        ┌─────────┐                              │
│                        │ 21/22/23│  OBELISK                     │
│                        │ IGNORE! │  (moving object,             │
│                        └─────────┘   don't use for              │
│                                      localization)              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

Tag IDs:
  24 = Red Goal    ← USE for localization
  20 = Blue Goal   ← USE for localization
  21 = Obelisk Face 1  ⎤
  22 = Obelisk Face 2  ⎥ IGNORE - robot moves these!
  23 = Obelisk Face 3  ⎦
```

#### Camera Setup

```java
// Create localizer
aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, "Webcam 1");

// Configure camera position on robot (IMPORTANT!)
aprilTagLocalizer.setCameraPosition(
    152.4,  // 6 inches forward of robot center (in mm)
    0,      // Centered left-right
    203.2   // 8 inches up from ground (in mm)
);
```

**Camera Mounting Diagram:**

```
              CAMERA
                ▼
     ┌─────────[■]─────────┐
     │          ↑          │
     │     zOffset (up)    │
     │          │          │
     │    ●─────┼──────────│──▶ xOffset (forward)
     │  robot   │          │
     │  center  │          │
     │          │          │
     └──────────┴──────────┘
                │
                └──▶ yOffset (left, usually 0)
```

#### Key Methods

```java
// Get all detected tags
List<AprilTagDetection> detections = localizer.getDetections();

// Get only goal tags (filters out obelisk)
List<AprilTagDetection> goals = localizer.getGoalDetections();

// Get estimated robot pose from tags
Pose2d visionPose = localizer.getEstimatedPose(alliance);

// Check if we have a recent valid detection
if (localizer.hasRecentDetection(200)) {  // 200ms
    // Use the vision pose
}
```

---

### PathFollower

**File:** `pickle/path/PathFollower.java`

Provides Road Runner-style path following with waypoints.

#### Fluent Path Building API

```java
// Build a path using the fluent API
pathFollower.buildPath()
    .from(startPose)                    // Starting position
    .setSpeed(0.6)                      // 60% speed
    .forward(1000)                      // 1000mm forward
    .strafeRight(500)                   // 500mm right
    .turnTo(Math.toRadians(90))         // Turn to face 90°
    .lineToHeading(x, y, heading)       // Go to position with heading
    .lineToFacing(x, y, faceX, faceY)   // Go to position, face a target
    .waitSeconds(0.5)                   // Pause for 0.5 seconds
    .build();                           // Start following
```

#### Heading Modes

```java
// FIXED: Maintain a specific heading
new Waypoint(x, y, Math.toRadians(45))
    .setHeadingMode(Waypoint.HeadingMode.FIXED);

// PATH: Face the direction of travel
new Waypoint(x, y)
    .setHeadingMode(Waypoint.HeadingMode.PATH);

// TARGET: Face a specific point (great for shooting!)
new Waypoint(x, y)
    .setHeadingTarget(goalPosition)
    .setHeadingMode(Waypoint.HeadingMode.TARGET);

// MAINTAIN: Keep current heading unchanged
new Waypoint(x, y)
    .setHeadingMode(Waypoint.HeadingMode.MAINTAIN);
```

#### Example: Shooting Path

```java
// Build path to shooting position while facing the goal
pathFollower.buildPath()
    .from(odometry.getPose())
    .setSpeed(MAX_DRIVE_SPEED)
    .setTolerance(40, Math.toRadians(3))  // 40mm, 3° tolerance
    .lineToFacing(
        shootingPosition.getX(),
        shootingPosition.getY(),
        goalCenter.getX(),      // Face toward goal
        goalCenter.getY()
    )
    .build();
```

---

## Tuning Guide

### Step 1: Measure Robot Dimensions

| Parameter | How to Measure | Typical Value |
|-----------|---------------|---------------|
| Wheel Radius | Measure diameter ÷ 2 | 48mm (GoBILDA) |
| Ticks Per Rev | Motor spec sheet | 537.7 (312 RPM) |
| Track Width | Left wheel center to right wheel center | 300-400mm |
| Wheel Base | Front wheel center to back wheel center | 250-350mm |

### Step 2: Tune Odometry

1. **Test Forward Motion:**
   - Command robot to drive forward 1000mm
   - Measure actual distance traveled
   - Adjust `WHEEL_RADIUS_MM` if different

2. **Test Rotation:**
   - Command robot to rotate 360°
   - Measure actual rotation
   - Adjust `TRACK_WIDTH_MM` if overshooting/undershooting

3. **Test Strafe:**
   - Command robot to strafe 1000mm
   - Measure actual distance
   - Adjust strafe multiplier in MecanumDriveHelper

### Step 3: Tune AprilTag Fusion

Start with these values and adjust:

```java
// Conservative starting values
final double APRILTAG_BLEND_FACTOR = 0.3;    // Trust odometry more initially
final long APRILTAG_MAX_AGE_MS = 200;        // Ignore stale readings
final double APRILTAG_CLOSE_RANGE_MM = 1500; // ~5 feet
```

**Signs you need higher blend factor:**
- Robot drifts off course over time
- Final position is consistently offset

**Signs you need lower blend factor:**
- Robot jerks when AprilTag is detected
- Position jumps around erratically

### Step 4: Tune Path Following

```java
// PID gains in MecanumDriveHelper
driveHelper.setPIDGains(
    0.003,  // translationKp - power per mm of error
    2.0     // headingKp - power per radian of error
);

// Tolerances
final double POSITION_TOLERANCE_MM = 40.0;  // How close is "arrived"
final double HEADING_TOLERANCE_DEG = 3.0;   // How aligned is "aligned"
```

---

## Troubleshooting

### Problem: Robot drifts to one side

**Possible causes:**
1. Wheel dimensions not measured correctly
2. Motor directions incorrect
3. One encoder not working

**Debug steps:**
```java
// Add to telemetry to check encoders
telemetry.addData("Encoders", odometry.getDiagnostics());
// Should show all 4 encoders changing when driving
```

### Problem: Robot spins when it should drive straight

**Possible cause:** Track width is wrong

**Fix:** Measure track width more carefully, or empirically adjust:
- Robot spins too much → **increase** track width
- Robot spins too little → **decrease** track width

### Problem: AprilTag corrections cause jerky motion

**Possible cause:** Blend factor too high

**Fix:** Reduce blend factor:
```java
final double APRILTAG_BLEND_FACTOR = 0.2;  // More conservative
```

### Problem: Robot overshoots waypoints

**Possible causes:**
1. Speed too high
2. Position tolerance too tight
3. PID gains too aggressive

**Fix:**
```java
// Reduce approach speed
final double APPROACH_SPEED = 0.25;  // Lower speed near target

// Increase tolerance
final double POSITION_TOLERANCE_MM = 60.0;  // More forgiving
```

### Problem: AprilTags not detected

**Checklist:**
- [ ] Camera is configured in Driver Station hardware config
- [ ] Camera has clear line of sight to tags
- [ ] Lighting is adequate (not too dark or too bright)
- [ ] Distance is within range (< 6 feet recommended)

---

## File Reference

| File | Location | Purpose |
|------|----------|---------|
| `PickleAutoHolonomic.java` | `pickle/` | Main autonomous OpMode |
| `MecanumOdometry.java` | `pickle/odometry/` | Encoder-based position tracking |
| `AprilTagLocalizer.java` | `pickle/vision/` | AprilTag detection and localization |
| `PathFollower.java` | `pickle/path/` | Road Runner-style path following |
| `Waypoint.java` | `pickle/path/` | Individual path waypoint |
| `MecanumDriveHelper.java` | `pickle/drive/` | Field-centric drive control |
| `Pose2d.java` | `pickle/geometry/` | Position + heading representation |
| `Translation2d.java` | `pickle/geometry/` | 2D position representation |
| `DecodeField.java` | `pickle/field/` | DECODE field positions and helpers |

---

## Learning Path

### Beginner

1. **Understand coordinate systems**
   - [FTC Coordinate System](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)

2. **Learn about encoders**
   - [gm0: Encoders](https://gm0.org/en/latest/docs/software/concepts/encoders.html)

3. **Understand mecanum kinematics**
   - [gm0: Mecanum Drives](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)

### Intermediate

4. **Study AprilTags**
   - [FTC AprilTag Introduction](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag-intro/apriltag-intro.html)

5. **Learn odometry concepts**
   - [gm0: Odometry](https://gm0.org/en/latest/docs/software/concepts/odometry.html)

6. **Understand PID control**
   - [CTRL ALT FTC: PID Introduction](https://www.ctrlaltftc.com/the-pid-controller)

### Advanced

7. **Study Road Runner**
   - [Road Runner Documentation](https://rr.brott.dev/docs/v1-0/tuning/)
   - [Learn Road Runner](https://learnroadrunner.com/)

8. **Explore sensor fusion**
   - [Kalman Filters (optional)](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

9. **Reference implementations**
   - [BunyipsFTC Library](https://github.com/Murray-Bridge-Bunyips/BunyipsLib)
   - [Pedro Pathing](https://pedropathing.com/)

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────┐
│                    HOLONOMIC AUTO QUICK REFERENCE               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  INIT:                                                          │
│    • Press X = BLUE alliance                                    │
│    • Press B = RED alliance                                     │
│    • Check telemetry for IMU and AprilTag status                │
│                                                                 │
│  DURING MATCH:                                                  │
│    • Robot follows path to shooting position                    │
│    • Settles and aligns to goal                                 │
│    • Fires 3 shots                                              │
│    • Parks in designated area                                   │
│                                                                 │
│  KEY CONSTANTS (PickleAutoHolonomic.java):                      │
│    MAX_DRIVE_SPEED = 0.6        (60% power)                     │
│    APPROACH_SPEED = 0.35        (35% near target)               │
│    POSITION_TOLERANCE_MM = 40   (40mm = "arrived")              │
│    APRILTAG_BLEND_FACTOR = 0.4  (40% vision trust)              │
│                                                                 │
│  TELEMETRY SHOWS:                                               │
│    • Current state (FOLLOW_PATH, SETTLE, SHOOT, etc.)           │
│    • Pose (X, Y, heading)                                       │
│    • Path progress (waypoint X of Y)                            │
│    • Distance to target                                         │
│    • AprilTag detection status                                  │
│    • Encoder values                                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
