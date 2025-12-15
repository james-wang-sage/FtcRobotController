# Pickle Motion System Overview

This document explains how our custom motion control system works, compares it to the popular Road Runner library, and helps you understand when each approach is appropriate.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Components Explained](#components-explained)
- [How It Works](#how-it-works)
- [Comparison with Road Runner](#comparison-with-road-runner)
- [Pros and Cons](#pros-and-cons)
- [When to Use Each Approach](#when-to-use-each-approach)
- [Learning Path](#learning-path)

---

## Architecture Overview

Our motion system consists of five main components:

```
┌─────────────────────────────────────────────────────────────────┐
│                        PathFollower                              │
│   (High-level path execution with waypoints)                    │
└───────────────────────────┬─────────────────────────────────────┘
                            │ uses
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     MecanumDriveHelper                          │
│   (Field-centric/robot-centric drive control)                   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ reads from
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      MecanumOdometry                            │
│   (Position tracking using wheel encoders + IMU)                │
└───────────────────────────┬─────────────────────────────────────┘
                            │ uses
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│               Geometry Classes (Pose2d, Translation2d)          │
│   (Mathematical representation of positions and poses)          │
└─────────────────────────────────────────────────────────────────┘
```

---

## Components Explained

### 1. Geometry Classes

**Location:** `pickle/geometry/`

#### Translation2d
Represents a 2D position (x, y) in millimeters.

```java
// Create a position
Translation2d position = new Translation2d(1000, 500); // 1000mm X, 500mm Y

// Calculate distance between points
double distance = position.distanceTo(otherPosition);

// Math operations
Translation2d sum = position.plus(otherPosition);
Translation2d scaled = position.times(0.5);
```

#### Pose2d
Represents a complete robot state: position + heading.

```java
// Create a pose (position + orientation)
Pose2d pose = new Pose2d(1000, 500, Math.toRadians(90)); // Facing +Y

// Get components
double x = pose.getX();           // 1000 mm
double y = pose.getY();           // 500 mm
double heading = pose.getHeading(); // 1.57 radians (90 degrees)

// Mirror for opposite alliance
Pose2d bluePose = redPose.mirrorForBlue();
```

**Coordinate System:**
```
                    +Y (toward far side)
                        ↑
                        │
                        │
    Blue Alliance ──────┼────── Red Alliance  +X
                        │
                        │
                    -Y (toward audience)

    Heading: 0 = facing +X, counter-clockwise positive
```

---

### 2. MecanumOdometry

**Location:** `pickle/odometry/MecanumOdometry.java`

Tracks the robot's position using wheel encoders and optionally fuses IMU heading for better accuracy.

#### How It Works

Mecanum wheels have a unique property: each wheel contributes to motion in a predictable way. By measuring how much each wheel has rotated, we can calculate how the robot moved.

```
Forward kinematics (robot-relative movement):
  ΔX = (FL + FR + BL + BR) / 4      ← Forward/backward
  ΔY = (-FL + FR + BL - BR) / 4     ← Left/right strafe
  Δθ = (-FL + FR - BL + BR) / (4 × trackWidth)  ← Rotation
```

#### Usage

```java
// Initialize in init()
MecanumOdometry odometry = new MecanumOdometry(fl, fr, bl, br, imu);
odometry.setWheelParameters(
    48.0,   // Wheel radius in mm (GoBILDA mecanum)
    537.7,  // Encoder ticks per revolution
    350.0,  // Track width (left-right distance)
    300.0   // Wheel base (front-back distance)
);
odometry.resetPose(new Pose2d(0, 0, 0)); // Starting position

// Update every loop
odometry.update();
Pose2d currentPose = odometry.getPose();

// Correct with AprilTag when available
if (aprilTagVisible) {
    odometry.correctPose(aprilTagPose, 0.3); // 30% trust in AprilTag
}
```

---

### 3. MecanumDriveHelper

**Location:** `pickle/drive/MecanumDriveHelper.java`

Handles the math for driving a mecanum robot in different control modes.

#### Control Modes

**Robot-Centric:** Inputs are relative to robot orientation
- "Forward" = direction robot is facing
- Simple but disorienting when robot rotates

```java
// +drive = robot's forward, +strafe = robot's right
driveHelper.driveRobotCentric(0.5, 0.3, 0.1);
```

**Field-Centric:** Inputs are relative to field orientation
- "Forward" = always +Y on field
- Intuitive for drivers, requires IMU

```java
// +fieldX = toward Red alliance, +fieldY = toward far side
driveHelper.driveFieldCentric(0.5, 0.3, 0.1);
```

#### Proportional Control to Target

The helper can automatically drive to a target pose using proportional control:

```java
// Drive toward target pose
driveHelper.driveToPose(targetPose, currentPose, maxSpeed);

// Check if arrived
if (driveHelper.isAtPose(target, current, 50, Math.toRadians(5))) {
    // Within 50mm and 5 degrees of target
}
```

---

### 4. PathFollower

**Location:** `pickle/path/PathFollower.java`

Executes a sequence of waypoints, managing speed, heading, and actions at each point.

#### Waypoint Types

```java
// Simple position waypoint
new Waypoint(1000, 500)

// Position with fixed heading
new Waypoint(1000, 500, Math.toRadians(90))

// With heading mode
new Waypoint(1000, 500)
    .setHeadingMode(HeadingMode.PATH)  // Face direction of travel

// With action at waypoint
new Waypoint(1000, 500)
    .setAction(() -> claw.close())
    .setActionDelay(0.5)  // Wait 0.5s before running action
```

#### Fluent Path Builder

```java
PathFollower follower = new PathFollower(driveHelper);

follower.buildPath()
    .from(startPose)                    // Starting position
    .setSpeed(0.6)                      // 60% speed
    .forward(500)                       // 500mm forward
    .strafeRight(300)                   // 300mm right
    .turnTo(Math.toRadians(90))         // Turn to face 90°
    .lineTo(1200, 800)                  // Move to absolute position
    .lineToHeading(1500, 1000, 0)       // Move with specific heading
    .addAction(() -> arm.raise())       // Run action at previous waypoint
    .waitSeconds(0.5)                   // Pause for 0.5 seconds
    .build();                           // Start following

// In loop
while (!follower.isComplete()) {
    odometry.update();
    follower.update(odometry.getPose());
}
```

---

## How It Works

### The Control Loop

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Loop (~50Hz)                         │
├─────────────────────────────────────────────────────────────┤
│  1. Update Odometry                                          │
│     └─> Read encoders → Calculate deltas → Update pose       │
│                                                              │
│  2. PathFollower.update(currentPose)                         │
│     └─> Check if waypoint reached                            │
│         └─> If yes: execute action, advance to next          │
│         └─> If no: calculate target pose for current waypoint│
│                                                              │
│  3. DriveHelper.driveToPose(target, current, speed)         │
│     └─> Calculate error (target - current)                   │
│     └─> Apply proportional control: power = Kp × error       │
│     └─> Transform to field-centric if needed                 │
│     └─> Send powers to motors                                │
└─────────────────────────────────────────────────────────────┘
```

### Proportional Control

Our system uses **P-control** (proportional control) to reach targets:

```
power = Kp × error

Where:
  error = target_position - current_position
  Kp = proportional gain (typically 0.02 to 0.05)
```

**Example:** If target X = 1000mm, current X = 800mm, and Kp = 0.03:
```
error = 1000 - 800 = 200mm
power = 0.03 × 200 = 0.6 (clamped to max speed)
```

As the robot approaches the target, error decreases, and so does power—creating smooth deceleration.

---

## Comparison with Road Runner

### What is Road Runner?

[Road Runner](https://learnroadrunner.com/) is a sophisticated motion planning library developed by ACME Robotics. It's widely used by competitive FTC teams for precise autonomous routines.

### Feature Comparison

| Feature | Our Implementation | Road Runner |
|---------|-------------------|-------------|
| **Trajectory Type** | Waypoint-based | Continuous spline curves |
| **Motion Profiling** | None (constant speed) | Full (velocity, acceleration, jerk limits) |
| **Path Following** | Proportional control | Advanced feedforward + feedback (PID) |
| **Heading Control** | 4 modes (fixed, path, target, maintain) | Spline-interpolated headings |
| **Localization** | Encoder + IMU | 2-wheel or 3-wheel dead wheels + IMU |
| **Tuning Required** | Minimal (~3 values) | Extensive (10+ parameters) |
| **Setup Time** | ~30 minutes | 2-8 hours |
| **Learning Curve** | Gentle | Steep |
| **Documentation** | This doc + Javadocs | Comprehensive website |
| **Competition Ready** | Beginner/Intermediate | Advanced/Elite |

### Motion Profile Comparison

**Our System (Constant Speed):**
```
Speed
  ↑
  │    ┌────────────────┐
  │    │                │
  │    │   max speed    │
  │────┘                └────
  └─────────────────────────→ Distance
       Start           End
```

**Road Runner (Trapezoidal/S-Curve Profile):**
```
Speed
  ↑
  │        ┌────────┐
  │       /│        │\
  │      / │        │ \
  │     /  │        │  \
  │────/   │        │   \────
  └─────────────────────────→ Distance
     Accel   Cruise   Decel
```

Road Runner's motion profiling means:
- Smoother starts and stops
- No wheel slip
- More consistent positioning
- But requires careful tuning

### Code Comparison

**Our PathFollower:**
```java
follower.buildPath()
    .forward(1000)
    .strafeRight(500)
    .turnTo(Math.toRadians(90))
    .build();
```

**Road Runner Trajectory:**
```java
Trajectory trajectory = drive.trajectoryBuilder(startPose)
    .forward(1000)
    .strafeRight(500)
    .build();

drive.followTrajectory(trajectory);
drive.turn(Math.toRadians(90));
```

Similar API, but Road Runner handles velocity/acceleration automatically.

---

## Pros and Cons

### Our Custom Implementation

#### Pros

| Advantage | Description |
|-----------|-------------|
| **Easy to Understand** | Simple proportional control, no complex math |
| **Quick Setup** | Works in ~30 minutes with minimal tuning |
| **Easy to Debug** | Fewer moving parts, straightforward troubleshooting |
| **Flexible** | Easy to modify for specific needs |
| **No External Dependencies** | All code is in your TeamCode folder |
| **Good for Learning** | Teaches fundamental robotics concepts |
| **Lightweight** | Minimal code overhead, fast compilation |
| **Team Ownership** | You understand every line of code |

#### Cons

| Disadvantage | Description |
|--------------|-------------|
| **No Motion Profiling** | Abrupt speed changes can cause wheel slip |
| **Less Accurate** | Proportional-only control has steady-state error |
| **No Spline Paths** | Can't create smooth curved trajectories |
| **Manual Timing** | Actions require manual timing/sequencing |
| **Limited Precision** | ~50mm position tolerance typical |
| **Drift Over Time** | Odometry accumulates error without vision correction |

### Road Runner

#### Pros

| Advantage | Description |
|-----------|-------------|
| **Motion Profiling** | Smooth acceleration/deceleration |
| **High Precision** | ~5-15mm position accuracy typical |
| **Spline Trajectories** | Smooth curved paths |
| **Feedforward Control** | Predictive motor commands |
| **Dead Wheel Support** | Best-in-class localization |
| **Async Actions** | Run actions during trajectories |
| **Competition Proven** | Used by World Championship teams |
| **GUI Dashboard** | Real-time visualization |

#### Cons

| Disadvantage | Description |
|--------------|-------------|
| **Steep Learning Curve** | Requires understanding of control theory |
| **Extensive Tuning** | 2-8 hours of calibration required |
| **Complex Debugging** | Many parameters to adjust when things go wrong |
| **Dead Wheel Requirement** | Best results need dedicated tracking wheels |
| **Overkill for Simple Tasks** | Complexity not always needed |
| **Version Compatibility** | Updates can break existing code |
| **Black Box** | Hard to understand internal workings |

---

## When to Use Each Approach

### Use Our Custom System When:

- ✅ Your team is new to FTC programming
- ✅ You need to get autonomous working quickly
- ✅ Your autonomous is simple (a few waypoints)
- ✅ You want to learn robotics fundamentals
- ✅ You don't have dead wheel odometry pods
- ✅ ~50mm position accuracy is acceptable
- ✅ You have limited build/testing time

### Use Road Runner When:

- ✅ Your team has programming experience
- ✅ You need high precision (<20mm accuracy)
- ✅ Your autonomous has complex curved paths
- ✅ You have time for thorough tuning (multiple sessions)
- ✅ You have dead wheel odometry pods installed
- ✅ You're competing at high-level regionals/worlds
- ✅ You want motion profiling for smooth movement

### Upgrade Path

Start with our system, then consider Road Runner when:
1. You've mastered the basics of autonomous
2. Position accuracy is limiting your scoring
3. You've installed dead wheel pods
4. You have time to tune (2+ sessions)

---

## Implementation Details

### Tuning Our System

Only three main values to tune:

```java
// In MecanumDriveHelper
private double strafeMultiplier = 1.1;  // Strafe compensation (1.0-1.4)
private double headingKp = 0.02;        // Heading P gain
private double translationKp = 0.03;   // Position P gain
```

**Tuning Process:**

1. **Strafe Multiplier:** Command 500mm strafe, measure actual distance
   - If robot moves 450mm: multiplier = 500/450 = 1.11

2. **Translation Kp:** Start at 0.02, increase until oscillation, then reduce by 20%

3. **Heading Kp:** Start at 0.02, increase until oscillation, then reduce by 20%

### Waypoint Tolerances

```java
new Waypoint(x, y)
    .setPositionTolerance(50)    // mm - how close is "arrived"
    .setHeadingTolerance(Math.toRadians(5));  // how aligned is "aligned"
```

- Tighter tolerances = more precision but slower completion
- Looser tolerances = faster but less accurate

---

## Learning Path

### Beginner (Start Here)

1. **Understand Coordinate Systems**
   - [FTC Field Coordinate System](https://ftc-docs.firstinspires.org/)
   - Practice converting between degrees and radians

2. **Learn Basic Motor Control**
   - Study `MecanumDriveHelper.driveRobotCentric()`
   - Understand how mecanum wheel powers combine

3. **Try Simple Autonomous**
   - Write a 3-waypoint autonomous
   - Debug using telemetry

### Intermediate

4. **Field-Centric Driving**
   - Understand IMU integration
   - Implement field-centric teleop

5. **Odometry Deep Dive**
   - Study `MecanumOdometry.update()`
   - Understand forward kinematics

6. **Path Following**
   - Create multi-waypoint paths
   - Use actions at waypoints

### Advanced

7. **PID Control Theory**
   - [PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)
   - Implement PID instead of P-only

8. **Sensor Fusion**
   - Combine odometry with AprilTag
   - Understand Kalman filtering concepts

9. **Road Runner Migration**
   - [LearnRoadRunner.com](https://learnroadrunner.com/)
   - Install and tune Road Runner

---

## Resources

### Our Implementation
- Source code in `pickle/` package
- Javadocs in each class

### Road Runner
- [LearnRoadRunner.com](https://learnroadrunner.com/) - Official tutorial
- [Road Runner GitHub](https://github.com/acmerobotics/road-runner)
- [FTC Discord #road-runner](https://discord.com/invite/first-tech-challenge)

### General FTC Programming
- [FTC Docs](https://ftc-docs.firstinspires.org/)
- [Game Manual 0](https://gm0.org/)
- [FTC Javadocs](https://javadoc.io/doc/org.firstinspires.ftc)

### Control Theory
- [PID Control - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [Motion Profiling - CTRL ALT FTC](https://www.ctrlaltftc.com/)

---

## Summary

Our custom motion system provides a **beginner-friendly** approach to autonomous control that prioritizes:

- **Understandability** over sophistication
- **Quick setup** over maximum precision
- **Learning fundamentals** over competition optimization

It's the right choice for teams learning FTC programming or with limited tuning time. As your team grows, Road Runner offers a clear upgrade path for teams needing higher precision.

**Remember:** A simple system you understand is better than a complex system you don't!
