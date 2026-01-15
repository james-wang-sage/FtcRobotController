# IMU in TeleOp Mode Guide

> **Field-centric driving** using the IMU makes holonomic robots much easier to control during driver-operated periods.

## Table of Contents

1. [Overview](#overview)
2. [Field-Centric vs Robot-Centric](#field-centric-vs-robot-centric)
3. [How It Works](#how-it-works)
4. [Implementation](#implementation)
5. [Advanced Features](#advanced-features)
6. [Troubleshooting](#troubleshooting)
7. [Learning Path](#learning-path)

---

## Overview

The IMU (Inertial Measurement Unit) in the REV Control Hub provides real-time orientation data that enables **field-centric driving** — one of the most impactful driver experience improvements for holonomic drivetrains.

### Why Use IMU in TeleOp?

| Without IMU | With IMU |
|-------------|----------|
| Robot-centric controls | Field-centric controls |
| Driver must mentally rotate inputs | Intuitive - "up" always means "away" |
| Harder when robot faces driver | Consistent regardless of orientation |
| Steeper learning curve | Easier for new drivers |

---

## Field-Centric vs Robot-Centric

### Visual Comparison

```
ROBOT-CENTRIC:                      FIELD-CENTRIC (with IMU):

    Robot facing →                      Robot facing →
    Driver pushes stick ↑               Driver pushes stick ↑
    Robot moves → (right)               Robot moves ↑ (forward on field)

         ↑ stick                             ↑ stick
         │                                   │
    ┌────┴────┐                         ┌────┴────┐
    │  ROBOT  │──→ movement             │  ROBOT  │
    │    →    │    (confusing!)         │    →    │
    └─────────┘                         └─────────┘
                                             │
                                             ↓ movement (intuitive!)
```

### The Math Behind It

Field-centric driving works by rotating the joystick input vector by the negative of the robot's heading:

```
┌──────────────────────────────────────────────────────────────────┐
│                    COORDINATE TRANSFORM                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Given:                                                         │
│     • Joystick input: (x, y)                                     │
│     • Robot heading: θ (from IMU)                                │
│                                                                  │
│   Field-centric output:                                          │
│     • rotX = x × cos(-θ) - y × sin(-θ)                           │
│     • rotY = x × sin(-θ) + y × cos(-θ)                           │
│                                                                  │
│   This rotates the input vector to compensate for robot rotation │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## How It Works

### Data Flow

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Gamepad    │────▶│   Rotate by  │────▶│   Mecanum    │
│  Joysticks   │     │  IMU Heading │     │    Math      │
│  (x, y, rx)  │     │              │     │              │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                            │                    │
                     ┌──────┴───────┐            │
                     │     IMU      │            │
                     │   Heading    │            ▼
                     └──────────────┘     ┌──────────────┐
                                          │    Motor     │
                                          │   Powers     │
                                          │ (FL,FR,BL,BR)│
                                          └──────────────┘
```

### IMU Orientation Setup

The IMU must know how it's mounted on your robot:

```
             USB Port Direction
                    │
                    ▼
            ┌───────────────┐
            │   ┌───────┐   │
            │   │  REV  │   │
            │   │  Hub  │   │
            │   │       │   │
            │   │ LOGO  │   │  ◀── Logo Direction
            │   │   ▲   │   │
            │   └───────┘   │
            │               │
            └───────────────┘
```

Common configurations:
- **Logo UP, USB FORWARD** - Hub mounted flat, USB toward front
- **Logo UP, USB LEFT** - Hub mounted flat, USB toward left side
- **Logo FORWARD, USB UP** - Hub mounted vertically

---

## Implementation

### Basic Field-Centric TeleOp

```java
package org.firstinspires.ftc.teamcode.pickle;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric TeleOp", group = "StarterBot")
public class FieldCentricTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Allows driver to reset "forward" direction
    private double headingOffset = 0;

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust for your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Tip", "Press Y to reset heading");
    }

    @Override
    public void loop() {
        // ===== READ INPUTS =====
        double y = -gamepad1.left_stick_y;   // Forward/back (inverted)
        double x = gamepad1.left_stick_x;    // Strafe left/right
        double rx = gamepad1.right_stick_x;  // Rotation

        // ===== GET ROBOT HEADING =====
        double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS) - headingOffset;

        // ===== FIELD-CENTRIC TRANSFORM =====
        // Rotate joystick inputs by negative heading
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // ===== MECANUM MATH =====
        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // ===== APPLY POWERS =====
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // ===== HEADING RESET =====
        // Press Y to set current direction as "forward"
        if (gamepad1.y) {
            headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // ===== TELEMETRY =====
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
        telemetry.addData("Stick", "(%.2f, %.2f)", x, y);
        telemetry.addData("Rotated", "(%.2f, %.2f)", rotX, rotY);
    }
}
```

### Using MecanumDriveHelper

If using the Pickle library's `MecanumDriveHelper`:

```java
@TeleOp(name = "Pickle TeleOp", group = "StarterBot")
public class PickleTeleOp extends OpMode {

    private MecanumDriveHelper driveHelper;
    private IMU imu;

    @Override
    public void init() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        DcMotor backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, PickleHardwareNames.IMU_NAME);
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        // Create drive helper with IMU
        driveHelper = new MecanumDriveHelper(frontLeft, frontRight, backLeft, backRight, imu);
        driveHelper.setFieldCentric(true);  // Enable field-centric mode
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Drive with field-centric control
        driveHelper.drive(forward, strafe, rotate);

        // Reset heading on button press
        if (gamepad1.y) {
            imu.resetYaw();
        }

        telemetry.addData("Mode", "Field-Centric");
        telemetry.addData("Heading", "%.1f°", driveHelper.getHeadingDegrees());
    }
}
```

---

## Advanced Features

### 1. Heading Hold (Auto-Straighten)

Automatically maintains heading when driver isn't rotating:

```java
private double targetHeading = 0;
private boolean holdingHeading = false;
private static final double HEADING_KP = 0.02;  // Proportional gain

@Override
public void loop() {
    double rx = gamepad1.right_stick_x;
    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    if (Math.abs(rx) > 0.1) {
        // Driver is rotating - update target
        holdingHeading = false;
        targetHeading = currentHeading;
    } else {
        // Driver released rotation stick - hold heading
        if (!holdingHeading) {
            holdingHeading = true;
            targetHeading = currentHeading;
        }
        // Calculate correction
        double error = normalizeAngle(targetHeading - currentHeading);
        rx = error * HEADING_KP;  // P controller
    }

    // Use rx in mecanum calculations...
}
```

### 2. Snap-to-Angle

Press a button to snap to nearest 90° angle:

```java
if (gamepad1.dpad_up) {
    // Snap to nearest 90° angle
    double current = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    double target = Math.round(current / 90.0) * 90.0;
    // Use PID to rotate to target...
}
```

### 3. Slow Mode Toggle

```java
private boolean slowMode = false;
private boolean lastBumper = false;

@Override
public void loop() {
    // Toggle slow mode
    if (gamepad1.left_bumper && !lastBumper) {
        slowMode = !slowMode;
    }
    lastBumper = gamepad1.left_bumper;

    double speedMultiplier = slowMode ? 0.3 : 1.0;

    // Apply to motor powers...
    frontLeft.setPower(frontLeftPower * speedMultiplier);
    // ...
}
```

### 4. Anti-Tip Detection

```java
@Override
public void loop() {
    double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    double roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

    if (Math.abs(pitch) > 30 || Math.abs(roll) > 30) {
        telemetry.addData("WARNING", "Robot tipping! Pitch=%.1f° Roll=%.1f°", pitch, roll);
        // Optionally reduce power or stop
    }
}
```

---

## Why resetYaw() is OK in TeleOp

```
┌──────────────────────────────────────────────────────────────────┐
│           resetYaw() in TeleOp vs Autonomous                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   TELEOP:                          AUTONOMOUS:                   │
│   ────────                         ───────────                   │
│   • Only need relative heading     • Need absolute field coords  │
│   • "Forward" is wherever          • "Forward" must be consistent│
│     driver wants it                  with field coordinate system│
│   • resetYaw() is fine!            • resetYaw() breaks odometry! │
│                                                                  │
│   In TeleOp, the driver defines    In Auto, the field defines    │
│   what "forward" means by          what "forward" means based    │
│   pressing the reset button.       on starting position.         │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## Troubleshooting

### Problem: Robot drives in wrong direction

**Cause:** Motor directions or IMU orientation incorrect

**Fix:**
1. Test each motor individually to verify directions
2. Check IMU orientation matches physical mounting:
```java
new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,     // Check this!
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD  // Check this!
)
```

### Problem: Field-centric is rotated 90°

**Cause:** IMU USB direction is wrong

**Fix:** Change `UsbFacingDirection` to match actual mounting:
- `FORWARD` - USB points toward robot front
- `BACKWARD` - USB points toward robot back
- `LEFT` - USB points toward robot left
- `RIGHT` - USB points toward robot right

### Problem: Heading drifts over time

**Cause:** IMU gyro drift (normal behavior)

**Fix:**
1. Let IMU warm up for 30 seconds before match
2. Use heading reset button to recalibrate during match
3. Consider using AprilTag corrections (advanced)

### Problem: Jerky movement

**Cause:** IMU readings may be noisy

**Fix:** Apply a low-pass filter:
```java
private double filteredHeading = 0;
private static final double FILTER_ALPHA = 0.8;

double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
filteredHeading = FILTER_ALPHA * filteredHeading + (1 - FILTER_ALPHA) * rawHeading;
```

---

## Learning Path

### Beginner

1. **Understand coordinate systems**
   - [FTC Coordinate System](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)

2. **Learn about the IMU**
   - [REV IMU Documentation](https://docs.revrobotics.com/duo-control/sensors/i2c/imu)
   - [FTC IMU Guide](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)

3. **Mecanum drive basics**
   - [gm0: Mecanum Drives](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)

### Intermediate

4. **Field-centric theory**
   - [CTRL ALT FTC: Field Centric](https://www.ctrlaltftc.com/practical-examples/drivetrain-control)

5. **PID control for heading hold**
   - [CTRL ALT FTC: PID Controller](https://www.ctrlaltftc.com/the-pid-controller)

### Advanced

6. **Sensor fusion**
   - Combine IMU with wheel encoders
   - Use AprilTags for drift correction

7. **Reference implementations**
   - [BunyipsFTC Library](https://github.com/Murray-Bridge-Bunyips/BunyipsLib)
   - [FTCLib Mecanum](https://docs.ftclib.org/ftclib/features/drivebases)

---

## Quick Reference Card

```
┌──────────────────────────────────────────────────────────────────┐
│                FIELD-CENTRIC TELEOP QUICK REFERENCE              │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  CONTROLS:                                                       │
│    Left Stick Y  = Forward/Backward                              │
│    Left Stick X  = Strafe Left/Right                             │
│    Right Stick X = Rotate                                        │
│    Y Button      = Reset heading (set current as "forward")      │
│                                                                  │
│  FIELD-CENTRIC TRANSFORM:                                        │
│    rotX = x * cos(-θ) - y * sin(-θ)                              │
│    rotY = x * sin(-θ) + y * cos(-θ)                              │
│                                                                  │
│  MECANUM MOTOR POWERS:                                           │
│    FL = (rotY + rotX + rx) / max                                 │
│    BL = (rotY - rotX + rx) / max                                 │
│    FR = (rotY - rotX - rx) / max                                 │
│    BR = (rotY + rotX - rx) / max                                 │
│                                                                  │
│  IMU ORIENTATION:                                                │
│    • Logo direction: Which way REV logo faces                    │
│    • USB direction: Which way USB port faces                     │
│    • Must match physical mounting!                               │
│                                                                  │
│  DRIVER TIPS:                                                    │
│    • Reset heading when robot faces away from you                │
│    • "Up" on stick = robot moves away from driver station        │
│    • Rotation is always relative to robot                        │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```
