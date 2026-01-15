# Mecanum Drive Guide for FTC Robotics

## Table of Contents
1. [What Are Mecanum Wheels?](#what-are-mecanum-wheels)
2. [How Mecanum Wheels Work](#how-mecanum-wheels-work)
3. [Why Use Mecanum Drive?](#why-use-mecanum-drive)
4. [Hardware Setup](#hardware-setup)
5. [Understanding Mecanum Kinematics](#understanding-mecanum-kinematics)
6. [Programming Approaches](#programming-approaches)
7. [TeleOp Implementation](#teleop-implementation)
8. [Autonomous Programming](#autonomous-programming)
9. [Field-Centric Drive](#field-centric-drive)
10. [Troubleshooting](#troubleshooting)
11. [Advanced Topics](#advanced-topics)

---

## What Are Mecanum Wheels?

**Mecanum wheels** are a specialized type of omni-directional wheel that enables holonomic movement. Each wheel has small rollers mounted at 45° angles around its circumference, allowing the robot to:
- Move forward/backward
- Strafe left/right
- Rotate in place
- Move diagonally
- **Combine all motions simultaneously**

### Key Characteristics
- **Rollers angled at 45°** to the wheel axis
- **Two types**: Left-handed (/) and right-handed (\) wheels
- **Requires 4 motors** for full holonomic control
- **Trade-off**: Maneuverability vs. traction

---

## How Mecanum Wheels Work

### The Physics

Each mecanum wheel can produce force in two directions:
1. **Normal rolling** (like a regular wheel)
2. **Angled sliding** (via the 45° rollers)

By combining forces from all four wheels, the robot can move in any direction.

### Visual Representation (Top View)

```
Front of Robot
     ↑

FL ↙ ↖ FR          FL = Front Left (left-handed wheel)
                   FR = Front Right (right-handed wheel)
BL ↗ ↘ BR          BL = Back Left (right-handed wheel)
                   BR = Back Right (left-handed wheel)
```

**Critical**: The rollers must form an **X pattern** when viewed from above, with all rollers pointing toward the center of the robot.

### Wheel Handedness

- **Left-handed (/)**: Rollers angle top-left to bottom-right
  - Used on: Front-Left and Back-Right positions
- **Right-handed (\)**: Rollers angle top-right to bottom-left
  - Used on: Front-Right and Back-Left positions

**IMPORTANT**: Installing wheels with the wrong handedness will prevent proper movement!

---

## Why Use Mecanum Drive?

### Advantages Over Tank/Differential Drive

| Capability | Tank Drive | Mecanum Drive |
|------------|------------|---------------|
| **Forward/Backward** | ✓ Excellent | ✓ Good |
| **Strafing** | ✗ Cannot strafe | ✓ Can strafe |
| **Rotation** | ✓ Can rotate | ✓ Can rotate |
| **Combined Motion** | ✗ Sequential only | ✓ Simultaneous |
| **Autonomous Speed** | Slower (turn-move-turn) | Faster (direct paths) |
| **Traction** | Excellent | Moderate |
| **Pushing Power** | Excellent | Moderate |
| **Programming Complexity** | Simple | Moderate |

### When Mecanum Excels

1. **Precise Positioning**
   - Align game pieces without rotating entire robot
   - Strafe into scoring position while maintaining orientation
   - Navigate tight spaces

2. **Field-Centric Control**
   - Driver pushes forward → robot moves forward on field (regardless of robot orientation)
   - More intuitive driving
   - Faster reaction times

3. **Faster Autonomous**
   - Direct diagonal paths instead of turn-move-turn sequences
   - Smoother trajectories
   - Time savings: 30-50% in typical autonomous routines

4. **Manipulator Alignment**
   - Keep arm/claw/shooter pointed at target while repositioning
   - Circle-strafe around game elements
   - Evasive maneuvers while maintaining offensive orientation

### Trade-offs to Consider

**Disadvantages**:
- Reduced traction (~70% efficiency due to 45° rollers)
- Can be pushed sideways more easily in defense
- More complex programming
- Higher cost (4 motors required)
- Wheels wear over time (rollers can deteriorate)

---

## Hardware Setup

### Required Components

#### 1. Mecanum Wheels (4 total)
- **2 left-handed wheels** (rollers angle /)
  - Install on: Front-Left and Back-Right positions
- **2 right-handed wheels** (rollers angle \)
  - Install on: Front-Right and Back-Left positions
- **Common FTC sizes**: 75mm, 96mm, 100mm
- **Popular options**:
  - goBILDA 96mm Mecanum Wheels
  - REV Robotics 75mm Mecanum Wheels
  - AndyMark 4" Mecanum Wheels

#### 2. Drive Motors (4 total)
- **Recommended motors**:
  - goBILDA 5203 series (Yellow Jacket motors)
    - 5203-2402-0019 (312 RPM) - Good all-around choice
    - 5203-2402-0014 (435 RPM) - Higher speed option
  - REV HD Hex Motors
  - REV Core Hex Motors
- **All 4 motors should be identical** for consistent performance
- **Motor encoders**: Built into most FTC motors (for autonomous)

#### 3. Control System & Motor Controllers

**IMPORTANT: For mecanum drive + launcher, you NEED an Expansion Hub!**

##### Motor Port Requirements
- **Mecanum drivetrain**: 4 motors
- **Launcher mechanism**: 1 motor
- **Total motors needed**: 5 motors minimum

##### Control Hub Capacity
- **Control Hub**: 4 motor ports (not enough for 5+ motors)
- **Expansion Hub**: Adds 4 additional motor ports (8 total)

##### Required Hardware
- **1x REV Control Hub** (main controller)
  - 4 motor ports
  - 6 servo ports
  - Built-in IMU (gyroscope + accelerometer)
  - 4 I2C ports, 8 digital I/O, 4 analog inputs
  - WiFi/Bluetooth for Driver Station connection

- **1x REV Expansion Hub** (required for 5+ motors)
  - 4 additional motor ports (8 total)
  - 6 additional servo ports (12 total)
  - Additional sensor ports
  - Connects to Control Hub via RS485 cable

##### Recommended Configuration
```
CONTROL HUB (Motor Ports 0-3):
├── Motor 0: front_left (mecanum drive)
├── Motor 1: front_right (mecanum drive)
├── Motor 2: back_left (mecanum drive)
└── Motor 3: back_right (mecanum drive)

EXPANSION HUB (Motor Ports 0-3):
├── Motor 0: launcher (scoring mechanism)
├── Motor 1: (available for future mechanisms)
├── Motor 2: (available for future mechanisms)
└── Motor 3: (available for future mechanisms)
```

**Why this configuration?**
- Keeps all drivetrain motors on the **Control Hub** for optimal performance
- Minimizes latency for drive controls (most time-critical)
- Leaves Expansion Hub ports for mechanisms that don't need instant response
- Easier to debug (drivetrain isolated on one hub)

##### Connection Setup
1. **Power both hubs** with XT30 connectors from battery
2. **Connect Expansion Hub to Control Hub** using RS485 cable (included)
3. **Configure in Driver Station** app:
   - Control Hub: Parent device
   - Expansion Hub: Child device (automatically detected)
4. **Update firmware** on both hubs to matching versions

#### 4. Power System
- **12V Robot Battery** (REV Slim Battery recommended)
  - 3000mAh capacity minimum
  - XT30 connectors for hubs
- **Battery Charger** (REV or compatible)
- **Power Distribution**:
  - Main switch (for safe power cutoff)
  - XT30 splitter or power cables for both hubs

#### 5. IMU (Inertial Measurement Unit)
- **Built into Control Hub** (no additional purchase needed!)
- **Used for**:
  - Field-centric drive (highly recommended)
  - Autonomous heading control
  - Rotation tracking
- **Alternative**: External BNO055 IMU (if Control Hub IMU damaged)

#### 6. Additional Components
- **Robot Controller Device** (Android phone, built into Control Hub)
- **Driver Station Device** (Android phone or tablet)
- **USB cables** for programming/debugging
- **XT30 connectors** and wire for power distribution
- **Mounting hardware** for hubs and motors
- **Structural components** (chassis, frame)

### Cost Estimate (Hardware Only)

| Component | Quantity | Approx. Cost (USD) |
|-----------|----------|-------------------|
| Control Hub | 1 | $250 |
| **Expansion Hub** | **1** | **$150** |
| Mecanum Wheels (set of 4) | 1 set | $80-120 |
| Drive Motors (Yellow Jacket) | 4 | $240 ($60 each) |
| Launcher Motor | 1 | $60 |
| Battery + Charger | 1 set | $100 |
| Cables & Connectors | - | $30 |
| **TOTAL (minimum)** | - | **~$910-950** |

**Note**: Prices are approximate and vary by supplier. Check REV Robotics, goBILDA, and AndyMark for current pricing.

### Why You Need the Expansion Hub

**Scenario: Mecanum + Launcher (5 motors total)**

| Configuration | Control Hub Motors | Expansion Hub Motors | Total Ports | Works? |
|--------------|-------------------|---------------------|-------------|---------|
| **Control Hub Only** | 4 (drive) + 1 (launcher) = 5 needed | - | 4 available | ❌ **NO** - Not enough ports! |
| **Control Hub + Expansion** | 4 (drive) | 1 (launcher) + 3 free | 8 available | ✅ **YES** - 3 ports left for future! |

**Additional Benefits of Expansion Hub**:
- **Future-proofing**: Room for intake, lift, arm, or other mechanisms
- **More servo ports**: 12 total (vs 6 with Control Hub only)
- **Additional sensors**: Extra I2C, analog, and digital ports
- **Redundancy**: If one hub fails, robot might still partially operate

### Alternative Configuration (NOT Recommended)

Some teams ask: "Can I split the drivetrain?"

**Possible but problematic**:
```
CONTROL HUB:
├── Motor 0: front_left
├── Motor 1: front_right
├── Motor 2: back_left
└── Motor 3: launcher

EXPANSION HUB:
└── Motor 0: back_right (one drive motor)
```

**Why this is bad**:
- ❌ Drivetrain split across two hubs → slight timing differences
- ❌ More complex code (different hardware maps)
- ❌ Harder to debug motor issues
- ❌ Potential for communication delays affecting drive smoothness
- ❌ No real cost savings (you still need the Expansion Hub)

**Bottom line**: If you're using 5+ motors, buy the Expansion Hub and do it right!

### Wheel Installation

**Critical**: Verify wheel orientation!

```
Top View - CORRECT Pattern:

    FRONT
     ↑
  ↙   ↖        ← Rollers point inward (forms X)
FL       FR

BL       BR
  ↗   ↘        ← Rollers point inward (forms X)
```

**Test**: When viewed from above, the four wheels' rollers should form an **X** pattern pointing toward the robot's center.

### Motor Configuration

In the **Driver Station** app, configure motors:

```
Control Hub Motor Ports:
- Motor 0: "front_left"   (goBILDA 5203 motor)
- Motor 1: "front_right"  (goBILDA 5203 motor)
- Motor 2: "back_left"    (goBILDA 5203 motor)
- Motor 3: "back_right"   (goBILDA 5203 motor)

IMU Sensors:
- I2C Port 0: "imu" (Built-in IMU)
```

**Note**: Exact port assignments can vary - just ensure your code matches your configuration.

---

## Understanding Mecanum Kinematics

### The Math Behind Movement

Each motor's power is calculated from three components:
- **Y**: Forward/backward motion
- **X**: Left/right strafing
- **RX**: Rotation (turning)

### Kinematic Equations

```
frontLeft  = Y + X + RX
frontRight = Y - X - RX
backLeft   = Y - X + RX
backRight  = Y + X - RX
```

### Why These Equations Work

Let's break down each movement:

**1. Forward Motion (Y = 1.0, X = 0, RX = 0)**
```
FL = 1 + 0 + 0 = 1.0  → All wheels forward
FR = 1 - 0 - 0 = 1.0
BL = 1 - 0 + 0 = 1.0
BR = 1 + 0 - 0 = 1.0
```

**2. Strafe Right (Y = 0, X = 1.0, RX = 0)**
```
FL = 0 + 1 + 0 =  1.0  → FL & BR forward
FR = 0 - 1 - 0 = -1.0  → FR & BL backward
BL = 0 - 1 + 0 = -1.0
BR = 0 + 1 - 0 =  1.0
```
The 45° rollers convert this into rightward motion!

**3. Rotate Clockwise (Y = 0, X = 0, RX = 1.0)**
```
FL = 0 + 0 + 1 =  1.0  → Left side forward
FR = 0 - 0 - 1 = -1.0  → Right side backward
BL = 0 - 0 + 1 =  1.0
BR = 0 + 0 - 1 = -1.0
```

**4. Diagonal (Y = 1.0, X = 1.0, RX = 0)**
```
FL = 1 + 1 + 0 =  2.0  → Forward-right diagonal
FR = 1 - 1 - 0 =  0.0
BL = 1 - 1 + 0 =  0.0
BR = 1 + 1 - 0 =  2.0
```
After normalization, only FL and BR run at full power → diagonal motion!

### Power Normalization

Since the equations can produce values > 1.0 or < -1.0, we must **normalize**:

```java
double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                     Math.max(Math.abs(backLeft), Math.abs(backRight))));

if (max > 1.0) {
    frontLeft /= max;
    frontRight /= max;
    backLeft /= max;
    backRight /= max;
}
```

This preserves the **ratio** of wheel speeds while keeping all values in the legal range [-1.0, 1.0].

---

## Programming Approaches

### Approach 1: Basic Robot-Centric (Beginner)

Direction controls are relative to the **robot's front**.

```java
@TeleOp(name="Mecanum: Robot-Centric", group="Mecanum")
public class MecanumRobotCentric extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // IMPORTANT: Reverse right-side motors if needed
        // Test and adjust based on your robot's motor mounting
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set brake mode for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joysticks (with deadzone)
            double y = applyDeadzone(-gamepad1.left_stick_y);  // Forward/backward
            double x = applyDeadzone(gamepad1.left_stick_x);   // Strafe
            double rx = applyDeadzone(gamepad1.right_stick_x); // Rotation

            // Calculate motor powers using mecanum kinematics
            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            // Normalize powers
            double maxPower = Math.max(Math.abs(frontLeftPower),
                                      Math.max(Math.abs(frontRightPower),
                                      Math.max(Math.abs(backLeftPower),
                                               Math.abs(backRightPower))));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Telemetry
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.update();
        }
    }

    /**
     * Apply deadzone to joystick input to prevent drift
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < 0.05) {
            return 0.0;
        }
        return value;
    }
}
```

**Pros**: Simple, easy to understand
**Cons**: Direction changes as robot rotates (can be confusing for drivers)

---

### Approach 2: With Speed Control (Intermediate)

Add speed modes for precision vs. speed.

```java
@TeleOp(name="Mecanum: Speed Control", group="Mecanum")
public class MecanumSpeedControl extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double speedMultiplier = 0.7; // Default: normal speed

    @Override
    public void runOpMode() {
        // Initialize motors (same as before)
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Speed control with triggers
            if (gamepad1.right_bumper) {
                speedMultiplier = 0.3; // Precision mode (slow)
            } else if (gamepad1.left_bumper) {
                speedMultiplier = 1.0; // Turbo mode (full speed)
            } else {
                speedMultiplier = 0.7; // Normal mode
            }

            // Read joysticks with deadzone and apply speed multiplier
            double y = applyDeadzone(-gamepad1.left_stick_y) * speedMultiplier;
            double x = applyDeadzone(gamepad1.left_stick_x) * speedMultiplier;
            double rx = applyDeadzone(gamepad1.right_stick_x) * speedMultiplier;

            // Apply cubic response curve for better low-speed control
            y = Math.signum(y) * Math.pow(Math.abs(y), 1.5);
            x = Math.signum(x) * Math.pow(Math.abs(x), 1.5);
            rx = Math.signum(rx) * Math.pow(Math.abs(rx), 1.5);

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            // Normalize
            double maxPower = Math.max(Math.abs(frontLeftPower),
                                      Math.max(Math.abs(frontRightPower),
                                      Math.max(Math.abs(backLeftPower),
                                               Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Telemetry
            String speedMode = speedMultiplier == 0.3 ? "PRECISION" :
                              (speedMultiplier == 1.0 ? "TURBO" : "NORMAL");
            telemetry.addData("Speed Mode", speedMode);
            telemetry.addData("Multiplier", "%.1f", speedMultiplier);
            telemetry.addData("Controls", "LB=Turbo, RB=Precision");
            telemetry.update();
        }
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
```

**Pros**: Better control, precision mode for alignment
**Cons**: Still robot-centric

---

## TeleOp Implementation

### Best Practices for Driver Control

#### 1. Control Scheme Options

**Standard Layout**:
- Left stick Y: Forward/backward
- Left stick X: Strafe left/right
- Right stick X: Rotate

**Alternative Layout**:
- Left stick: Movement (Y=forward, X=strafe)
- Right stick X: Rotation
- Right stick Y: (unused or secondary function)

#### 2. Speed Profiles

```java
// Linear response
double power = joystick;

// Quadratic response (more precision at low speeds)
double power = Math.signum(joystick) * Math.pow(joystick, 2);

// Cubic response (even more precision)
double power = Math.pow(joystick, 3);

// Custom curve (1.5 exponent - good balance)
double power = Math.signum(joystick) * Math.pow(Math.abs(joystick), 1.5);
```

#### 3. Complete TeleOp Example

```java
@TeleOp(name="Mecanum: Competition TeleOp", group="Mecanum")
public class MecanumCompetitionTeleOp extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Speed settings
    private static final double PRECISION_SPEED = 0.3;
    private static final double NORMAL_SPEED = 0.7;
    private static final double TURBO_SPEED = 1.0;
    private static final double DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Determine speed mode
            double speedMultiplier = getSpeedMultiplier();

            // Get joystick inputs
            double y = getJoystickValue(-gamepad1.left_stick_y, speedMultiplier);
            double x = getJoystickValue(gamepad1.left_stick_x, speedMultiplier);
            double rx = getJoystickValue(gamepad1.right_stick_x, speedMultiplier);

            // Drive the robot
            driveRobotCentric(y, x, rx);

            // Update telemetry
            updateTelemetry(speedMultiplier);
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double getSpeedMultiplier() {
        if (gamepad1.right_bumper) return PRECISION_SPEED;
        if (gamepad1.left_bumper) return TURBO_SPEED;
        return NORMAL_SPEED;
    }

    private double getJoystickValue(double rawValue, double multiplier) {
        // Apply deadzone
        if (Math.abs(rawValue) < DEADZONE) return 0.0;

        // Apply response curve and speed multiplier
        double value = Math.signum(rawValue) * Math.pow(Math.abs(rawValue), 1.5);
        return value * multiplier;
    }

    private void driveRobotCentric(double y, double x, double rx) {
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Normalize
        double maxPower = Math.max(Math.abs(frontLeftPower),
                                  Math.max(Math.abs(frontRightPower),
                                  Math.max(Math.abs(backLeftPower),
                                           Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void updateTelemetry(double speedMultiplier) {
        String mode = speedMultiplier == PRECISION_SPEED ? "PRECISION" :
                     (speedMultiplier == TURBO_SPEED ? "TURBO" : "NORMAL");

        telemetry.addData("Speed Mode", mode);
        telemetry.addData("Controls", "LB=Turbo | RB=Precision");
        telemetry.addData("FL", "%.2f", frontLeft.getPower());
        telemetry.addData("FR", "%.2f", frontRight.getPower());
        telemetry.addData("BL", "%.2f", backLeft.getPower());
        telemetry.addData("BR", "%.2f", backRight.getPower());
        telemetry.update();
    }
}
```

---

## Autonomous Programming

### Time-Based Movement (Simple)

```java
@Autonomous(name="Mecanum: Time-Based Auto", group="Mecanum")
public class MecanumTimeBasedAuto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        initializeMotors();

        waitForStart();

        // Autonomous sequence
        driveForward(0.5, 2000);      // Forward 2 seconds
        pause(500);

        strafeRight(0.5, 1500);       // Strafe right 1.5 seconds
        pause(500);

        rotateClockwise(0.4, 1000);   // Rotate 1 second
        pause(500);

        driveBackward(0.5, 2000);     // Backward 2 seconds
        pause(500);

        strafeLeft(0.5, 1500);        // Strafe left 1.5 seconds
    }

    private void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Movement primitives
    private void driveForward(double power, long milliseconds) {
        setMotorPowers(power, power, power, power);
        sleep(milliseconds);
        stopMotors();
    }

    private void driveBackward(double power, long milliseconds) {
        setMotorPowers(-power, -power, -power, -power);
        sleep(milliseconds);
        stopMotors();
    }

    private void strafeRight(double power, long milliseconds) {
        setMotorPowers(power, -power, -power, power);
        sleep(milliseconds);
        stopMotors();
    }

    private void strafeLeft(double power, long milliseconds) {
        setMotorPowers(-power, power, power, -power);
        sleep(milliseconds);
        stopMotors();
    }

    private void rotateClockwise(double power, long milliseconds) {
        setMotorPowers(power, -power, power, -power);
        sleep(milliseconds);
        stopMotors();
    }

    private void rotateCounterClockwise(double power, long milliseconds) {
        setMotorPowers(-power, power, -power, power);
        sleep(milliseconds);
        stopMotors();
    }

    private void driveDiagonal(double y, double x, long milliseconds) {
        double fl = y + x;
        double fr = y - x;
        double bl = y - x;
        double br = y + x;

        setMotorPowers(fl, fr, bl, br);
        sleep(milliseconds);
        stopMotors();
    }

    // Helper methods
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private void pause(long milliseconds) {
        sleep(milliseconds);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }
}
```

**Pros**: Simple, easy to understand
**Cons**: Inaccurate (battery voltage affects distance), no feedback

---

### Encoder-Based Movement (Better)

```java
@Autonomous(name="Mecanum: Encoder-Based Auto", group="Mecanum")
public class MecanumEncoderAuto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constants - ADJUST THESE FOR YOUR ROBOT
    private static final double COUNTS_PER_MOTOR_REV = 537.7;  // goBILDA 5203-2402-0019
    private static final double WHEEL_DIAMETER_MM = 96.0;      // 96mm mecanum wheels
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);

    @Override
    public void runOpMode() {
        initializeMotorsWithEncoders();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Autonomous sequence (distances in millimeters)
        driveForward(600, 0.5);       // Drive forward 600mm
        strafeRight(400, 0.5);        // Strafe right 400mm
        rotateByAngle(90, 0.4);       // Rotate 90 degrees (approximate)
        driveBackward(600, 0.5);      // Drive backward 600mm
    }

    private void initializeMotorsWithEncoders() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Enable encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveForward(double distanceMM, double power) {
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);

        setTargetPositions(targetCounts, targetCounts, targetCounts, targetCounts);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPowers(power, power, power, power);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Status", "Driving forward");
            telemetry.addData("Target", targetCounts);
            telemetry.addData("Current", frontLeft.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveBackward(double distanceMM, double power) {
        driveForward(-distanceMM, power);
    }

    private void strafeRight(double distanceMM, double power) {
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM * 1.1); // Strafe multiplier

        setTargetPositions(targetCounts, -targetCounts, -targetCounts, targetCounts);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPowers(power, power, power, power);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Status", "Strafing right");
            telemetry.update();
        }

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeLeft(double distanceMM, double power) {
        strafeRight(-distanceMM, power);
    }

    private void rotateByAngle(double degrees, double power) {
        // This is approximate - use IMU for accurate rotation
        double distanceMM = (degrees / 360.0) * (Math.PI * 400); // Assume 400mm robot width
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);

        setTargetPositions(targetCounts, -targetCounts, targetCounts, -targetCounts);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPowers(power, power, power, power);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Status", "Rotating");
            telemetry.update();
        }

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper methods
    private void setTargetPositions(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private boolean motorsAreBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() &&
               backLeft.isBusy() && backRight.isBusy();
    }
}
```

**Pros**: More accurate than time-based
**Cons**: Still affected by wheel slip, needs calibration for strafing

---

## Field-Centric Drive

### What is Field-Centric?

**Robot-Centric**: Joystick controls relative to robot's front
- Robot rotates → control direction changes

**Field-Centric**: Joystick controls relative to field
- Robot rotates → control direction stays the same
- Forward on joystick = forward on field (always)

### Implementation with IMU

```java
@TeleOp(name="Mecanum: Field-Centric", group="Mecanum")
public class MecanumFieldCentric extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Ready - Press Options to reset heading");
        telemetry.update();

        waitForStart();

        // Reset IMU heading at start
        imu.resetYaw();

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = applyDeadzone(-gamepad1.left_stick_y);
            double x = applyDeadzone(gamepad1.left_stick_x);
            double rx = applyDeadzone(gamepad1.right_stick_x);

            // Get robot heading from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement vector by the robot's heading (field-centric transformation)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Optional: Reset heading with button
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Drive using transformed coordinates
            driveFieldCentric(rotY, rotX, rx);

            // Telemetry
            telemetry.addData("Mode", "Field-Centric");
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(botHeading));
            telemetry.addData("Reset Heading", "Press Options button");
            telemetry.update();
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");

        // Define hub orientation
        // ADJUST THESE based on how your Control Hub is mounted!
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);
    }

    private void driveFieldCentric(double y, double x, double rx) {
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Normalize
        double maxPower = Math.max(Math.abs(frontLeftPower),
                                  Math.max(Math.abs(frontRightPower),
                                  Math.max(Math.abs(backLeftPower),
                                           Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
```

### IMU Configuration

**Critical**: Set the IMU orientation correctly based on your Control Hub mounting.

```java
// Example 1: Hub flat, logo facing up, USB facing forward
new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
)

// Example 2: Hub on its side, logo facing left, USB facing up
new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
    RevHubOrientationOnRobot.UsbFacingDirection.UP
)

// Example 3: Hub upside-down, logo facing down, USB facing back
new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
)
```

---

## Troubleshooting

### Problem 1: Robot Doesn't Move Straight

**Symptoms**: When driving forward, robot drifts left or right

**Causes**:
1. Motors not reversed correctly
2. Wheels installed incorrectly
3. Uneven motor power (friction, wear)

**Solutions**:
```java
// Test each motor individually
if (gamepad1.a) frontLeft.setPower(0.5);   // Should spin forward
if (gamepad1.b) frontRight.setPower(0.5);  // Should spin forward
if (gamepad1.x) backLeft.setPower(0.5);    // Should spin forward
if (gamepad1.y) backRight.setPower(0.5);   // Should spin forward

// Reverse motors as needed
frontRight.setDirection(DcMotor.Direction.REVERSE);
backRight.setDirection(DcMotor.Direction.REVERSE);
// Or try reversing left side instead:
// frontLeft.setDirection(DcMotor.Direction.REVERSE);
// backLeft.setDirection(DcMotor.Direction.REVERSE);
```

### Problem 2: Strafing Moves at an Angle

**Symptoms**: Strafing right/left also moves forward/backward

**Causes**:
1. Wheels installed with wrong handedness
2. Motor directions incorrect
3. Mecanum wheel rollers worn

**Solutions**:
1. Verify wheel installation (should form X pattern from top)
2. Test strafe in isolation:
```java
// Pure strafe right
frontLeft.setPower(1.0);
frontRight.setPower(-1.0);
backLeft.setPower(-1.0);
backRight.setPower(1.0);
```
3. Check that rollers are in good condition

### Problem 3: Robot Spins Instead of Strafing

**Symptoms**: Attempting to strafe causes rotation

**Cause**: Front and back motors swapped in code

**Solution**:
```java
// Verify motor mapping
telemetry.addData("FL Port", "Motor 0");
telemetry.addData("FR Port", "Motor 1");
telemetry.addData("BL Port", "Motor 2");
telemetry.addData("BR Port", "Motor 3");

// Check hardware configuration matches code
frontLeft = hardwareMap.get(DcMotor.class, "front_left");   // Actually front-left?
frontRight = hardwareMap.get(DcMotor.class, "front_right"); // Actually front-right?
backLeft = hardwareMap.get(DcMotor.class, "back_left");     // Actually back-left?
backRight = hardwareMap.get(DcMotor.class, "back_right");   // Actually back-right?
```

### Problem 4: Unpredictable Movement

**Symptoms**: Robot moves erratically, doesn't respond to controls correctly

**Causes**:
1. Power normalization missing
2. Joystick deadzone too small (drift)
3. Multiple gamepad inputs conflicting

**Solutions**:
```java
// Always normalize
double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                     Math.max(Math.abs(bl), Math.abs(br))));
if (max > 1.0) {
    fl /= max; fr /= max; bl /= max; br /= max;
}

// Larger deadzone
private double applyDeadzone(double value) {
    return Math.abs(value) < 0.1 ? 0.0 : value;  // 0.1 instead of 0.05
}
```

### Problem 5: Field-Centric Drift

**Symptoms**: Field-centric mode gradually becomes incorrect over time

**Cause**: IMU gyro drift (accumulates small errors)

**Solutions**:
1. Add heading reset button
2. Use AprilTags for absolute heading correction
3. Accept minor drift (reset between matches)

```java
// Reset button
if (gamepad1.options) {
    imu.resetYaw();
}

// Or auto-reset when robot is stationary (advanced)
if (Math.abs(y) < 0.01 && Math.abs(x) < 0.01 && Math.abs(rx) < 0.01) {
    // Robot stationary for > 2 seconds → could reset
}
```

### Problem 6: Low Power/Slow Movement

**Symptoms**: Robot moves slowly even at full joystick

**Causes**:
1. Speed multiplier set too low
2. Over-normalization (dividing by values > 1 unnecessarily)
3. Battery voltage low

**Solutions**:
```java
// Check speed multiplier
telemetry.addData("Speed Multiplier", speedMultiplier);

// Check actual power values
telemetry.addData("FL Power", frontLeft.getPower());
telemetry.addData("Max Calculated", maxPower);

// Verify battery voltage
telemetry.addData("Battery", "%.1f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
```

---

## Advanced Topics

### 1. Mecanum Drive Class (Reusable)

Create a separate class for clean code organization:

```java
public class MecanumDrive {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private boolean fieldCentric;

    public MecanumDrive(HardwareMap hardwareMap, boolean useFieldCentric) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.fieldCentric = useFieldCentric;

        if (fieldCentric) {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
            imu.resetYaw();
        }
    }

    /**
     * Drive the robot
     * @param x Strafe (-1 to 1, positive = right)
     * @param y Forward (-1 to 1, positive = forward)
     * @param rx Rotation (-1 to 1, positive = clockwise)
     */
    public void drive(double x, double y, double rx) {
        double rotX = x;
        double rotY = y;

        if (fieldCentric && imu != null) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        }

        double fl = rotY + rotX + rx;
        double fr = rotY - rotX - rx;
        double bl = rotY - rotX + rx;
        double br = rotY + rotX - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                             Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void resetHeading() {
        if (fieldCentric && imu != null) {
            imu.resetYaw();
        }
    }

    public double getHeading() {
        if (fieldCentric && imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return 0.0;
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }
}

// Usage in OpMode:
@TeleOp(name="Mecanum: Using Drive Class")
public class MecanumWithClass extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, true); // true = field-centric

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive.drive(x, y, rx);

            if (gamepad1.options) {
                drive.resetHeading();
            }

            telemetry.addData("Heading", "%.1f°", drive.getHeading());
            telemetry.update();
        }
    }
}
```

### 2. Road Runner Integration

For competition-level autonomous with path planning:

**Installation**:
1. Follow instructions at [learnroadrunner.com](https://learnroadrunner.com)
2. Tune your robot's kinematics
3. Create trajectories

**Example**:
```java
@Autonomous(name="Mecanum: RoadRunner Auto")
public class MecanumRoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Define complex trajectory
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
            .forward(30)
            .strafeLeft(15)
            .splineTo(new Vector2d(20, 20), 0)
            .build();

        waitForStart();

        drive.followTrajectory(traj);
    }
}
```

### 3. PID Control for Autonomous

For precise movement with feedback:

```java
// Simple PID controller for rotation
public class PIDController {
    private double kP, kI, kD;
    private double integral = 0;
    private double previousError = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}

// Usage: Rotate to specific angle
PIDController headingController = new PIDController(0.01, 0, 0.001);

while (opModeIsActive()) {
    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    double targetHeading = 90.0; // degrees

    double rotationPower = headingController.calculate(targetHeading, currentHeading);

    drive(0, 0, rotationPower);

    if (Math.abs(targetHeading - currentHeading) < 1.0) {
        break; // Close enough
    }
}
```

### 4. Dead Wheel Odometry

For maximum autonomous accuracy, use dead wheels (unpowered tracking wheels):

**Benefits**:
- Not affected by wheel slip
- More accurate position tracking
- Essential for RoadRunner

**Setup**:
- Mount 2-3 unpowered omni wheels with encoders
- Position: 2 parallel (forward/backward tracking) + 1 perpendicular (strafe tracking)
- Integrate with RoadRunner's localizer

---

## Summary

### Key Takeaways

1. **Mecanum wheels provide holonomic motion** - move in any direction without rotating first
2. **Requires 4 motors** for 3 degrees of freedom (X, Y, rotation)
3. **Trade-off**: Maneuverability vs. traction
4. **Proper wheel installation is critical** - must form X pattern
5. **Field-centric mode** greatly improves driver control (requires IMU)
6. **Start simple** (robot-centric) and add complexity as needed

## Beginner Learning Path

This section provides a structured, step-by-step path for teams new to FTC and mecanum drive programming. Each phase builds on the previous one with clear goals, deliverables, and resources.

---

### Phase 0: Prerequisites (Before You Start)

**Goal**: Ensure your team has the foundation to succeed

**What to Learn**:
1. **Java Basics** (if completely new to programming)
   - Variables, data types (int, double, boolean)
   - If statements and loops (while, for)
   - Methods/functions
   - Basic object-oriented concepts (classes, objects)

2. **FTC Environment Setup**
   - Install Android Studio
   - Clone FtcRobotController repository
   - Connect to Robot Controller via WiFi
   - Build and deploy a sample OpMode

**Resources**:
- **Java for FTC**: [FTC Docs - Programming Resources](https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html)
- **Java Basics Tutorial**: [W3Schools Java](https://www.w3schools.com/java/)
- **Android Studio Setup**: [FTC Docs - Android Studio](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Installing-Android-Studio)
- **Video: "FTC Programming 101"**: Search YouTube for "FTC Programming Tutorial for Beginners"

**Deliverable**: Successfully run `BasicOpMode_Linear.java` sample on your robot

**Time Estimate**: 1-2 weeks (depending on prior programming experience)

---

### Phase 1: Basic Robot-Centric TeleOp (Weeks 1-2)

**Goal**: Get your mecanum robot driving with basic controls

**What to Build**:
1. Copy and customize the "Mecanum: Robot-Centric" OpMode from this guide
2. Configure hardware in Driver Station app
3. Test each motor individually to verify directions
4. Implement deadzone to prevent joystick drift

**Key Concepts**:
- Hardware mapping (`hardwareMap.get()`)
- Motor directions (FORWARD vs REVERSE)
- Mecanum kinematics equations
- Power normalization
- Telemetry for debugging

**Resources**:
- **This Guide**: See "Approach 1: Basic Robot-Centric" section above
- **FTC SDK Sample**: `FtcRobotController/src/.../samples/BasicOpMode_Linear.java`
- **Game Manual 0 - Mecanum Drive**: [gm0.org - Mecanum Drive](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)
- **Video: "How Mecanum Wheels Work"**: [YouTube - Mecanum Physics](https://www.youtube.com/results?search_query=mecanum+wheels+explained)
- **FTC Discord**: [Join here](https://discord.com/invite/first-tech-challenge) - #programming-help channel

**Practice Tasks**:
1. Drive forward/backward in a straight line (10 feet)
2. Strafe left/right without rotating
3. Rotate in place (360° spin)
4. Drive in a square pattern (combine movements)
5. Navigate through cones/obstacles

**Common Issues & Solutions**:
- Robot doesn't move → Check motor configuration names match code
- Robot spins when trying to go straight → Verify motor directions
- Robot drifts when joystick released → Increase deadzone value

**Deliverable**: TeleOp program where drivers can smoothly control robot in all directions

**Time Estimate**: 2 weeks (including practice time)

---

### Phase 2: Refinements and Speed Control (Weeks 3-4)

**Goal**: Make driving more precise and driver-friendly

**What to Add**:
1. Multiple speed modes (precision/normal/turbo)
2. Response curves for better low-speed control
3. Brake mode for precise stopping
4. Comprehensive telemetry display

**Key Concepts**:
- Trigger/button input for speed modes
- Mathematical response curves (linear, quadratic, cubic)
- Zero power behavior (BRAKE vs FLOAT)
- Telemetry formatting

**Resources**:
- **This Guide**: See "Approach 2: With Speed Control" section
- **Game Manual 0 - Driver Control**: [gm0.org - Driver Control](https://gm0.org/en/latest/docs/software/tutorials/teleop.html)
- **Video: "Advanced TeleOp Techniques"**: Search "FTC TeleOp Best Practices"

**Practice Tasks**:
1. Test different response curves and find what feels best
2. Practice precision driving (slow mode) for game piece pickup
3. Practice fast driving (turbo mode) for field traversal
4. Get comfortable switching between modes mid-match

**Deliverable**: Polished TeleOp with speed control that drivers prefer

**Time Estimate**: 2 weeks

---

### Phase 3: IMU Integration and Field-Centric (Weeks 5-7)

**Goal**: Implement field-centric control for more intuitive driving

**What to Learn**:
1. What an IMU (Inertial Measurement Unit) is and how it works
2. How to configure IMU based on Control Hub orientation
3. Coordinate transformation math (field → robot coordinates)
4. Handling IMU drift with reset buttons

**Key Concepts**:
- Sensors in FTC (IMU, gyroscope)
- Trigonometry (sin, cos) for rotation
- Radians vs degrees
- Reference frames (robot-centric vs field-centric)

**Resources**:
- **This Guide**: See "Field-Centric Drive" section
- **FTC SDK Sample**: `SensorIMUOrthogonal.java`
- **Game Manual 0 - IMU**: [gm0.org - IMU](https://gm0.org/en/latest/docs/software/tutorials/imu.html)
- **REV Hub IMU Docs**: [REV Robotics IMU Documentation](https://docs.revrobotics.com/duo-control/sensors/i2c/imu)
- **Video: "Field-Centric Drive Explained"**: Search "FTC Field Centric Drive Tutorial"
- **Math Refresher**: Khan Academy - Trigonometry basics

**Practice Tasks**:
1. Read IMU values and display heading on telemetry
2. Implement basic field-centric transformation
3. Drive robot while facing different directions (test if forward always goes field-forward)
4. Practice resetting heading with button press
5. Run mock match scenarios with field-centric enabled

**Common Issues & Solutions**:
- Heading drifts over time → Normal IMU behavior, add reset button
- Wrong directions when robot rotates → Check IMU hub orientation parameters
- Jittery movement → Verify sin/cos math uses radians (not degrees)

**Deliverable**: Field-centric TeleOp that drivers can use confidently in matches

**Time Estimate**: 3 weeks (math concepts may need extra time)

---

### Phase 4: Basic Autonomous (Weeks 8-10)

**Goal**: Create simple autonomous routines

**What to Build**:
1. Time-based autonomous (simple but less accurate)
2. Encoder-based autonomous (more accurate)
3. Basic autonomous sequence for competition

**Key Concepts**:
- Autonomous vs TeleOp OpMode structure
- Time-based movement (`sleep()` method)
- Motor encoders and position tracking
- Run modes (RUN_TO_POSITION, RUN_USING_ENCODER)
- Creating reusable movement methods

**Resources**:
- **This Guide**: See "Autonomous Programming" section
- **FTC SDK Sample**: `RobotAutoDriveByEncoder_Linear.java`
- **Game Manual 0 - Autonomous**: [gm0.org - Autonomous](https://gm0.org/en/latest/docs/software/tutorials/autonomous.html)
- **Video: "FTC Autonomous Programming"**: Search "FTC Autonomous Tutorial"
- **Encoder Explanation**: [Encoders Explained](https://gm0.org/en/latest/docs/software/concepts/encoders.html)

**Practice Tasks**:
1. Drive forward specific distance using encoders
2. Strafe to specific position
3. Create autonomous to park in designated zone
4. Build autonomous sequence: start → score preload → park
5. Test autonomous reliability (run 10 times, measure success rate)

**Calibration Steps**:
1. Measure your motor's encoder counts per revolution (COUNTS_PER_MOTOR_REV)
2. Measure wheel diameter accurately
3. Calculate COUNTS_PER_MM constant
4. Test forward movement over known distance, adjust if needed
5. Test strafing (usually needs 1.1x-1.2x multiplier due to slip)

**Common Issues & Solutions**:
- Robot doesn't drive exact distance → Calibrate encoder constants
- Strafing moves at angle → Add strafe multiplier (1.1-1.2x)
- Inconsistent movement → Check battery charge, use RUN_TO_POSITION mode

**Deliverable**: Working autonomous routine that scores and parks reliably

**Time Estimate**: 3 weeks

---

### Phase 5: Advanced Autonomous (Weeks 11-14) - OPTIONAL

**Goal**: Learn professional-grade autonomous programming

**What to Learn**:
1. Path planning with Road Runner library
2. Dead wheel odometry for accurate localization
3. PID control for precise movement
4. AprilTag detection for position correction

**Key Concepts**:
- Trajectory planning
- Localization (knowing where robot is on field)
- PID controllers (Proportional-Integral-Derivative)
- Computer vision integration
- State machines for complex autonomous

**Resources**:
- **Road Runner**: [learnroadrunner.com](https://learnroadrunner.com)
- **Road Runner Quickstart**: [GitHub Quickstart](https://github.com/acmerobotics/road-runner-quickstart)
- **Game Manual 0 - Road Runner**: [gm0.org - Road Runner](https://gm0.org/en/latest/docs/software/tutorials/road-runner.html)
- **PID Explained**: [PID Control Simplified](https://www.youtube.com/watch?v=wkfEZmsQqiA)
- **AprilTag Tutorial**: [FTC AprilTag Documentation](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html)
- **FTC ML/Vision**: [FIRST Machine Learning Toolchain](https://ftc-ml.firstinspires.org/)

**Practice Tasks**:
1. Install and configure Road Runner
2. Tune your robot's drive characteristics
3. Create spline trajectories for smooth paths
4. Implement dead wheel odometry
5. Use AprilTags for position correction mid-autonomous

**Deliverable**: Advanced autonomous with smooth paths and high accuracy

**Time Estimate**: 4+ weeks (significant complexity increase)

---

## Resource Library

### Official FTC Documentation
- **FTC Documentation Hub**: [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/)
- **FTC SDK JavaDocs**: [javadoc.io/doc/org.firstinspires.ftc](https://javadoc.io/doc/org.firstinspires.ftc)
- **FTC GitHub Repository**: [github.com/FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- **Game & Season Info**: [firstinspires.org/ftc](https://www.firstinspires.org/robotics/ftc)

### Community Resources
- **Game Manual 0** (GM0): [gm0.org](https://gm0.org/en/latest/)
  - Community-driven comprehensive guide
  - Excellent mecanum drive section
  - Hardware and software tutorials
  - Competition strategies

- **FTC Discord Server**: [discord.gg/first-tech-challenge](https://discord.com/invite/first-tech-challenge)
  - Very active community (#programming-help, #hardware-help)
  - Quick answers to questions
  - Event discussions

- **Chief Delphi**: [chiefdelphi.com/tag/ftc](https://www.chiefdelphi.com/tag/ftc)
  - Cross-FRC/FTC technical discussions
  - Build logs from top teams
  - Strategy discussions

### Video Resources
Search YouTube for these topics:
- "FTC Programming Tutorial for Beginners"
- "Mecanum Wheels Explained"
- "FTC Field Centric Drive"
- "How to use RoadRunner FTC"
- "FTC Autonomous Programming"
- "FTC OpMode Tutorial"

**Recommended Channels**:
- FIRST Tech Challenge (official)
- Dexter Industries
- Alan G. Smith (FTC tutorials)

### Hardware-Specific Documentation
- **REV Robotics**: [docs.revrobotics.com](https://docs.revrobotics.com/)
  - Control Hub, Expansion Hub documentation
  - Sensor guides (IMU, distance, color)
  - Motor and servo specs

- **goBILDA**: [gobildaproducts.com](https://www.gobildaproducts.com/)
  - Motor specifications
  - Mecanum wheel details
  - Build guides

### Programming Tools
- **FTC Dashboard**: [acmerobotics.github.io/ftc-dashboard](https://acmerobotics.github.io/ftc-dashboard/)
  - Real-time telemetry graphing
  - Live configuration editing
  - Essential for Road Runner tuning

- **OnBot Java**: Built into Robot Controller
  - Browser-based code editor
  - No Android Studio needed
  - Good for quick tests

### Competition Preparation
- **Game Manual Part 1**: [firstinspires.org/resource-library/ftc/game-and-season-info](https://www.firstinspires.org/resource-library/ftc/game-and-season-info)
  - Official competition rules
  - Robot construction rules
  - Game-specific regulations

- **Game Manual Part 2**: Technical rules and regulations
  - Robot inspection checklist
  - Safety requirements
  - Field specifications

### Practice Tools
- **FTC Simulator**: [github.com/Beta8397/virtual_robot](https://github.com/Beta8397/virtual_robot)
  - Test code without physical robot
  - Practice autonomous paths
  - Useful when robot isn't available

---

## Suggested Practice Schedule

### For Teams Meeting 2x Per Week (3-hour sessions)

**Weeks 1-2: Foundation**
- Session 1: Hardware setup, configure Driver Station
- Session 2: Run basic samples, understand OpMode structure
- Session 3: Copy robot-centric code, test motor directions
- Session 4: Debug and practice driving

**Weeks 3-4: Refinement**
- Session 1: Add speed control modes
- Session 2: Implement response curves and test
- Session 3: Practice driving, get driver feedback
- Session 4: Polish and prepare for scrimmage

**Weeks 5-7: Field-Centric**
- Session 1: Learn IMU basics, configure orientation
- Session 2: Implement field-centric transformation
- Session 3: Debug and test extensively
- Session 4: Practice driving field-centric
- Session 5: Compare robot vs field-centric, choose best
- Session 6: Final polish for competition

**Weeks 8-10: Autonomous**
- Session 1: Learn encoder basics, time-based movement
- Session 2: Implement encoder-based movement
- Session 3: Calibrate encoder constants
- Session 4: Build autonomous sequence
- Session 5: Test and refine reliability
- Session 6: Practice autonomous + driver handoff

### For Teams Meeting 3-4x Per Week
- Follow same structure but move faster
- Add extra practice sessions
- Consider splitting into sub-teams (drive, autonomous, mechanism)

---

## Getting Help

### When You're Stuck
1. **Check telemetry**: Add debug output to see what's happening
2. **Test incrementally**: Comment out code until you find the problem
3. **Read error messages**: Android Studio shows helpful compile errors
4. **Search GM0**: Someone else likely had same issue
5. **Ask on Discord**: FTC community is very helpful

### Good Questions Get Better Answers
**Bad question**: "My robot doesn't work, help!"

**Good question**:
> "Using mecanum drive, robot-centric OpMode. When I push left joystick forward, robot spins instead of moving forward. I've verified motor names match hardware config. Motors are: frontLeft (port 0), frontRight (port 1, REVERSED), backLeft (port 2), backRight (port 3, REVERSED). Here's my code: [paste relevant snippet]. What am I doing wrong?"

**Include**:
- What you're trying to do
- What's actually happening (unexpected behavior)
- What you've already tried
- Relevant code snippet
- Hardware setup details

---

## Milestones and Checkpoints

Use these milestones to track your team's progress:

### ✅ Checkpoint 1: Basic Movement (Week 2)
- [ ] All 4 motors spin correctly when tested individually
- [ ] Robot drives forward in straight line
- [ ] Robot can strafe left/right
- [ ] Robot can rotate in place
- [ ] Driver can control robot smoothly with gamepad

### ✅ Checkpoint 2: Refined TeleOp (Week 4)
- [ ] Speed modes implemented (precision/normal/turbo)
- [ ] Deadzone prevents drift
- [ ] Brake mode enabled for precise stopping
- [ ] Telemetry shows useful debug info
- [ ] Drivers prefer this code over basic version

### ✅ Checkpoint 3: Field-Centric (Week 7)
- [ ] IMU reads heading correctly
- [ ] Field-centric transformation works
- [ ] Reset button implemented
- [ ] Robot moves field-forward regardless of orientation
- [ ] Drivers can use effectively in practice

### ✅ Checkpoint 4: Competition-Ready TeleOp (Week 8)
- [ ] Code is reliable (no crashes in 20+ runs)
- [ ] Drivers are comfortable and confident
- [ ] All controls documented for drivers
- [ ] Backup OpMode available (in case of issues)

### ✅ Checkpoint 5: Basic Autonomous (Week 10)
- [ ] Encoder constants calibrated accurately
- [ ] Can drive forward/backward to specific distance (±2 inches)
- [ ] Can strafe to specific position (±3 inches)
- [ ] Autonomous sequence completes 8/10 times successfully
- [ ] Can score preload and park

### ✅ Checkpoint 6: Competition Autonomous (Week 12)
- [ ] Autonomous is reliable (9/10 success rate)
- [ ] Multiple autonomous options for different starting positions
- [ ] Autonomous integrates with TeleOp (shares hardware config)
- [ ] Tested under competition conditions (low battery, bumped robot, etc.)

---

## Next Steps After This Guide

Once you've mastered mecanum drive programming:

1. **Add Mechanisms**: Arm, claw, lift, intake
2. **Vision Processing**: AprilTag detection, color recognition
3. **Advanced Autonomous**: Road Runner, path following
4. **Multi-Robot Coordination**: Alliance partner communication
5. **Optimization**: Reduce cycle times, improve efficiency
6. **Competition Strategy**: Auto selection, endgame routines

**Remember**: Programming is iterative. Start simple, test often, and improve gradually!

---

*Updated Learning Path for beginner FTC teams - DECODE 2025-2026 season*

---

*Document created for FTC Team - DECODE 2025-2026 season*
*Focused on Mecanum Drive implementation*
