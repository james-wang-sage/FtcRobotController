# Omni Wheels Guide for FTC Robotics

## Table of Contents
1. [Introduction to Omni Wheels](#introduction-to-omni-wheels)
2. [Comparison: Omni Wheels vs Regular Wheels](#comparison-omni-wheels-vs-regular-wheels)
3. [Why 4 Motors Are Required](#why-4-motors-are-required)
4. [Control and Performance Advantages](#control-and-performance-advantages)
5. [Drive Configurations](#drive-configurations)
6. [Coding Approaches](#coding-approaches)
7. [Practical Implementation Examples](#practical-implementation-examples)
8. [Common Pitfalls and Best Practices](#common-pitfalls-and-best-practices)

---

## Introduction to Omni Wheels

**Omni wheels** (short for omni-directional wheels) are specialized wheels with small rollers mounted perpendicular to the wheel's circumference. These rollers allow the wheel to move freely in two directions:
- **Primary direction**: Normal rolling motion (like a regular wheel)
- **Secondary direction**: Sliding motion perpendicular to the wheel axis (via the rollers)

This dual-motion capability is what enables holonomic drive systems, allowing robots to move in any direction without rotating first.

### Types of Omni Wheels in FTC
- **Standard Omni Wheels**: Small rollers around the circumference
- **Mecanum Wheels**: Rollers angled at 45° (a special type of omni wheel)

---

## Comparison: Omni Wheels vs Regular Wheels

| Aspect | Regular Wheels | Omni Wheels |
|--------|----------------|-------------|
| **Degrees of Freedom** | 2 (forward/backward, rotation) | 3 (forward/backward, strafe, rotation) |
| **Movement Type** | Non-holonomic (must rotate to change direction) | Holonomic (can move in any direction instantly) |
| **Traction** | High traction in all directions | Reduced traction (due to rollers) |
| **Pushing Power** | Excellent for pushing matches | Moderate - can be pushed sideways |
| **Precision Positioning** | Requires turning arcs | Direct linear movement to any position |
| **Field Orientation** | Must rotate to face target | Can maintain constant heading while moving |
| **Complexity** | Simpler programming | More complex kinematics |
| **Typical Use Cases** | Tank drive, differential drive | X-drive, mecanum drive, H-drive |
| **Defense Capability** | Strong against pushing | Vulnerable to lateral pushes |
| **Autonomous Accuracy** | Requires turn-move-turn sequences | Direct path planning |

### When to Choose Omni Wheels
**Advantages:**
- Need to strafe (move sideways) during gameplay
- Precise positioning without rotating
- Field-centric driving (move relative to field, not robot)
- Faster autonomous routines (no turning required)
- Better for manipulator alignment

**Disadvantages:**
- Less pushing power in defense
- More complex programming
- Higher cost (need 4 motors minimum)
- Can be pushed sideways more easily

---

## Why 4 Motors Are Required

### The Physics of Holonomic Drive

For a robot to achieve **holonomic motion** (movement in any direction without rotation), it needs to independently control movement in multiple axes. Here's why 4 motors are necessary:

### Degrees of Freedom Analysis

A planar robot (moving on a flat field) has **3 degrees of freedom**:
1. **X-axis translation** (forward/backward)
2. **Y-axis translation** (left/right strafe)
3. **Rotation** (turning)

To control 3 degrees of freedom independently, you need **at least 3 independent actuators**. However, 4 motors provide:
- **Redundancy**: Better stability and control
- **Balanced Forces**: Even weight distribution and power
- **Symmetry**: Predictable behavior in all directions

### Why Not 3 Motors?

While theoretically possible with 3 omni wheels (arranged at 120° intervals), this configuration has significant drawbacks:
- Asymmetric movement (some directions faster than others)
- Complex kinematics calculations
- Uneven weight distribution
- Poor traction balance
- Rarely used in competitive FTC

### Why Not 2 Motors?

With only 2 motors, you can only control 2 degrees of freedom. This means:
- Either no strafing (like tank drive), OR
- No independent rotation control
- Cannot achieve true holonomic motion

### The 4-Motor Configurations

**Mecanum Drive** (most common in FTC):
```
[Front-Left]    [Front-Right]
    ↙  ↖            ↗  ↘

    ↗  ↘            ↙  ↖
[Back-Left]     [Back-Right]
```
- 4 mecanum wheels with rollers at 45°
- Each motor contributes to all 3 degrees of freedom
- Symmetric and balanced

**X-Drive** (also called holonomic drive):
```
      [Front]
     /      \
[Left]      [Right]
     \      /
      [Back]
```
- 4 omni wheels mounted at 45° to robot frame
- Excellent maneuverability
- More complex mounting

---

## Control and Performance Advantages

### 1. Holonomic Motion
**Regular wheels** require a "turn-move-turn" sequence:
```
1. Rotate to face target
2. Drive forward to target
3. Rotate to final orientation
```

**Omni wheels** move directly:
```
1. Drive diagonally while maintaining orientation
```

**Time savings**: Up to 40-60% faster autonomous routines

### 2. Field-Centric Driving

With omni wheels, you can implement **field-centric control** where:
- Pushing joystick forward ALWAYS moves robot toward opponent (regardless of robot rotation)
- Driver doesn't need to mentally translate robot orientation
- IMU (gyroscope) tracks robot heading
- Kinematics automatically adjust motor powers

**Example scenario**: Your robot is facing sideways, but you need to move forward on the field:
- **Regular wheels**: Driver must push joystick sideways
- **Omni wheels (field-centric)**: Driver pushes joystick forward, robot strafes

### 3. Manipulator Alignment

When using arms, claws, or shooters:
- **Regular wheels**: Must rotate entire robot to align
- **Omni wheels**: Strafe to position while keeping manipulator aimed

**Example**: Scoring in a basket:
```java
// Regular drive: Turn → Move → Turn
robot.turnTo(targetAngle);
robot.driveForward(distance);
robot.turnTo(scoreAngle);

// Omni drive: Direct movement
robot.driveFieldCentric(xTarget, yTarget, scoreAngle);
```

### 4. Evasive Maneuvers

In teleop, drivers can:
- Strafe away from opponents while maintaining offensive orientation
- Circle-strafe around game elements
- Dodge while keeping camera/sensors pointed forward

### 5. Precision Positioning

Omni wheels allow **true vector control**:
```java
// Move at 45° angle while rotating
drive(0.707, 0.707, 0.3);  // x, y, rotation
```

Regular wheels cannot achieve this motion without multiple steps.

---

## Drive Configurations

### 1. Mecanum Drive

**Configuration**: 4 mecanum wheels, rollers at 45°
- Front-left and back-right: Left-handed wheels (/)
- Front-right and back-left: Right-handed wheels (\)

**Kinematics**:
```
FL = x + y + rotation
FR = x - y - rotation
BL = x - y + rotation
BR = x + y - rotation
```

**Pros**:
- Standard FTC configuration
- Good documentation and samples
- Readily available wheels
- Moderate complexity

**Cons**:
- ~70% efficiency (due to 45° rollers)
- Loses traction under pushing
- Wear on rollers over time

### 2. X-Drive

**Configuration**: 4 omni wheels mounted at 45° to robot chassis

**Kinematics**:
```
FL = x + y + rotation
FR = -x + y - rotation
BL = -x + y + rotation
BR = x + y - rotation
```

**Pros**:
- Maximum maneuverability
- Faster than mecanum (100% efficiency in cardinal directions)
- Cool factor

**Cons**:
- More complex mounting
- Larger footprint (wheels stick out at corners)
- Less common (fewer examples)

### 3. H-Drive

**Configuration**: 3 motors for tank drive + 1 center strafe wheel

**Kinematics**:
```
Left = y + rotation
Right = y - rotation
Center = x
```

**Pros**:
- Simple to understand
- Full traction when driving straight
- Can use regular wheels for front/back

**Cons**:
- Only uses 3 motors (one motor idle during straight driving)
- Center wheel can be pushed off ground
- Asymmetric design

---

## Coding Approaches

### Approach 1: Direct Motor Control (Beginner)

Set motor powers directly based on joystick input:

```java
@TeleOp(name="Mecanum: Direct Control", group="Omni")
public class MecanumDirect extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // Reverse right side motors (depends on mounting)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Read joysticks
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers (mecanum kinematics)
            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            // Normalize powers (keep all values between -1 and 1)
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
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
```

**Pros**: Simple, easy to understand
**Cons**: Robot-centric only (direction changes as robot rotates)

---

### Approach 2: Field-Centric Control (Intermediate)

Use an IMU to maintain field orientation:

```java
@TeleOp(name="Mecanum: Field-Centric", group="Omni")
public class MecanumFieldCentric extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        // Reset heading at start
        imu.resetYaw();

        while (opModeIsActive()) {
            // Read joysticks
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Get robot heading (in radians)
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate movement vector by robot heading (field-centric transformation)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Reset heading with button press
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Calculate motor powers
            double frontLeftPower = rotY + rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double backLeftPower = rotY - rotX + rx;
            double backRightPower = rotY + rotX - rx;

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
            telemetry.addData("Heading", Math.toDegrees(botHeading));
            telemetry.addData("Mode", "Field-Centric (Press Options to reset)");
            telemetry.update();
        }
    }
}
```

**Pros**: Intuitive driving, direction independent of robot rotation
**Cons**: Requires IMU, more complex math

---

### Approach 3: Class-Based Abstraction (Advanced)

Create a reusable drive class:

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
     * @param x Strafe component (-1 to 1, positive = right)
     * @param y Forward component (-1 to 1, positive = forward)
     * @param rotation Rotation component (-1 to 1, positive = clockwise)
     */
    public void drive(double x, double y, double rotation) {
        double rotX = x;
        double rotY = y;

        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }

        double frontLeftPower = rotY + rotX + rotation;
        double frontRightPower = rotY - rotX - rotation;
        double backLeftPower = rotY - rotX + rotation;
        double backRightPower = rotY + rotX - rotation;

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

    /**
     * Drive to a specific field position (autonomous)
     */
    public void driveToPosition(double targetX, double targetY, double targetHeading) {
        // Implementation would use encoders and PID control
        // This is a placeholder for autonomous movement
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
}

// Usage in OpMode:
@TeleOp(name="Mecanum: Class-Based", group="Omni")
public class MecanumClassBased extends LinearOpMode {
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

**Pros**: Reusable, maintainable, clean OpModes
**Cons**: More initial setup, requires understanding of OOP

---

### Approach 4: RoadRunner Integration (Competition-Level)

For advanced autonomous with path planning:

```java
// Using RoadRunner library (not included in base SDK)
// See: https://learnroadrunner.com/

@Autonomous(name="Mecanum: RoadRunner Auto", group="Omni")
public class MecanumRoadRunner extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Define trajectory
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
            .splineTo(new Vector2d(30, 30), 0)
            .build();

        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectory(traj);
        }
    }
}
```

**Pros**: Professional-grade path planning, velocity constraints, accurate
**Cons**: Steep learning curve, requires tuning

---

## Practical Implementation Examples

### Example 1: Basic TeleOp with Speed Control

```java
@TeleOp(name="Mecanum: Speed Control", group="Omni")
public class MecanumSpeedControl extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double speedMultiplier = 1.0;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Speed control with triggers
            if (gamepad1.right_trigger > 0.1) {
                speedMultiplier = 0.3; // Slow mode (precision)
            } else if (gamepad1.left_trigger > 0.1) {
                speedMultiplier = 1.0; // Turbo mode
            } else {
                speedMultiplier = 0.7; // Normal mode
            }

            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * speedMultiplier;
            double rx = gamepad1.right_stick_x * speedMultiplier;

            // Mecanum kinematics
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

            telemetry.addData("Speed Mode", speedMultiplier == 0.3 ? "PRECISION" :
                                          (speedMultiplier == 1.0 ? "TURBO" : "NORMAL"));
            telemetry.addData("Multiplier", "%.1f", speedMultiplier);
            telemetry.update();
        }
    }
}
```

### Example 2: Autonomous Movement

```java
@Autonomous(name="Mecanum: Simple Auto", group="Omni")
public class MecanumSimpleAuto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to run with encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Autonomous sequence
        driveForward(0.5, 1000);    // Drive forward for 1 second
        sleep(500);
        strafeRight(0.5, 1000);     // Strafe right for 1 second
        sleep(500);
        rotateClockwise(0.5, 1000); // Rotate for 1 second
        sleep(500);
        driveBackward(0.5, 1000);   // Drive backward for 1 second
    }

    private void driveForward(double power, long milliseconds) {
        setAllMotors(power, power, power, power);
        sleep(milliseconds);
        stopMotors();
    }

    private void driveBackward(double power, long milliseconds) {
        setAllMotors(-power, -power, -power, -power);
        sleep(milliseconds);
        stopMotors();
    }

    private void strafeRight(double power, long milliseconds) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(milliseconds);
        stopMotors();
    }

    private void strafeLeft(double power, long milliseconds) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(milliseconds);
        stopMotors();
    }

    private void rotateClockwise(double power, long milliseconds) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(milliseconds);
        stopMotors();
    }

    private void setAllMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setAllMotors(0, 0, 0, 0);
    }
}
```

---

## Common Pitfalls and Best Practices

### Pitfall 1: Not Normalizing Motor Powers
**Problem**: When combined movements exceed motor capability, robot moves incorrectly.

**Example**:
```java
// WRONG: Can result in values > 1.0
double fl = 0.8 + 0.7 + 0.5; // = 2.0 (capped at 1.0 by hardware)
```

**Solution**: Always normalize:
```java
// CORRECT
double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                     Math.max(Math.abs(bl), Math.abs(br))));
if (max > 1.0) {
    fl /= max;
    fr /= max;
    bl /= max;
    br /= max;
}
```

### Pitfall 2: Incorrect Motor Directions
**Problem**: Motors run backward, causing unexpected movement.

**Solution**: Test each motor individually:
```java
// Test mode: Run one motor at a time
if (gamepad1.a) frontLeft.setPower(0.5);
if (gamepad1.b) frontRight.setPower(0.5);
if (gamepad1.x) backLeft.setPower(0.5);
if (gamepad1.y) backRight.setPower(0.5);

// Reverse motors as needed
frontRight.setDirection(DcMotor.Direction.REVERSE);
backRight.setDirection(DcMotor.Direction.REVERSE);
```

### Pitfall 3: Wrong Wheel Installation
**Problem**: Mecanum wheels installed incorrectly (rollers forming X instead of O when viewed from top).

**Correct pattern (top view)**:
```
     \ /
      X  ← Should form an X pattern (rollers point to center)
     / \
```

**Incorrect pattern**:
```
     / \
      X  ← Wrong! Forms a diamond
     \ /
```

### Pitfall 4: Ignoring Deadzone
**Problem**: Controller drift causes unwanted movement when joysticks are released.

**Solution**: Implement deadzone:
```java
double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;
```

### Pitfall 5: Field-Centric Drift
**Problem**: IMU drift causes field-centric mode to become inaccurate over time.

**Solution**: Add reset button and consider fusion with vision:
```java
if (gamepad1.options) {
    imu.resetYaw(); // Reset to current orientation
}

// Or use AprilTags for absolute heading correction
```

### Best Practice 1: Use Zero Power Behavior
```java
// Set brake mode for precise stopping
frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
```

### Best Practice 2: Add Telemetry for Debugging
```java
telemetry.addData("FL Power", frontLeft.getPower());
telemetry.addData("FR Power", frontRight.getPower());
telemetry.addData("BL Power", backLeft.getPower());
telemetry.addData("BR Power", backRight.getPower());
telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
telemetry.update();
```

### Best Practice 3: Implement Power Curves
For better control at low speeds:
```java
// Cubic response curve (more precise at low speeds)
double y = -Math.pow(gamepad1.left_stick_y, 3);
double x = Math.pow(gamepad1.left_stick_x, 3);
double rx = Math.pow(gamepad1.right_stick_x, 3);
```

### Best Practice 4: Document Your Configuration
Create a hardware configuration document:
```
Motor Configuration:
- front_left: Control Hub Motor Port 0 (goBILDA 5203-2402-0019)
- front_right: Control Hub Motor Port 1 (REVERSED)
- back_left: Control Hub Motor Port 2
- back_right: Control Hub Motor Port 3 (REVERSED)

Wheel Configuration:
- All wheels: goBILDA Mecanum 96mm
- Pattern: X-pattern (FL and BR same handedness)
```

---

## Beginner Learning Path

This section provides a structured, step-by-step path for teams new to FTC and omni-directional drive systems. Each phase builds on the previous one with clear goals, deliverables, and resources.

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

### Phase 1: Understanding Omni Wheel Types

**Goal**: Understand the differences between omni wheel configurations and choose the right one for your team

**What to Learn**:
1. **Mecanum vs Standard Omni vs X-Drive**
   - How each configuration works
   - Pros and cons of each
   - Which is best for your robot and game strategy

2. **Why 4 Motors Are Needed**
   - Degrees of freedom analysis
   - Physics of holonomic motion
   - Trade-offs vs tank drive

**Resources**:
- **This Guide**: See "Comparison: Omni Wheels vs Regular Wheels" and "Drive Configurations" sections
- **Game Manual 0 - Drive Types**: [gm0.org - Drivetrain Types](https://gm0.org/en/latest/docs/hardware/drivetrains.html)
- **Video: "Omni Wheels Explained"**: Search YouTube for "Omni Wheels vs Mecanum vs Tank Drive"
- **Video: "FTC Drivetrain Comparison"**: Search "FTC Drivetrain Pros and Cons"

**Practice Tasks**:
1. Watch videos of different drive types in competition
2. Discuss as a team which configuration suits your strategy
3. Document your decision and reasoning

**Deliverable**: Team decision on which omni drive configuration to use (recommend mecanum for beginners)

**Time Estimate**: 1 week (research and discussion)

---

### Phase 2: Basic Robot-Centric TeleOp (Weeks 1-2)

**Goal**: Get your omni-wheel robot driving with basic controls

**What to Build**:
1. Copy and customize a basic drive OpMode for your configuration
2. Configure hardware in Driver Station app
3. Test each motor individually to verify directions
4. Implement deadzone to prevent joystick drift

**Key Concepts**:
- Hardware mapping (`hardwareMap.get()`)
- Motor directions (FORWARD vs REVERSE)
- Kinematics equations (mecanum/X-drive specific)
- Power normalization
- Telemetry for debugging

**Resources**:
- **This Guide**: See "Coding Approaches" section for your drive type
- **Mecanum-Specific**: See companion `Mecanum-Drive-Guide.md` in this folder
- **FTC SDK Sample**: `BasicOpMode_Linear.java`
- **Game Manual 0**: [gm0.org - Programming Drivetrain](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)
- **FTC Discord**: [Join here](https://discord.com/invite/first-tech-challenge) - #programming-help channel

**Practice Tasks** (for Mecanum):
1. Drive forward/backward in a straight line (10 feet)
2. Strafe left/right without rotating
3. Rotate in place (360° spin)
4. Drive in a square pattern (combine movements)
5. Navigate through cones/obstacles

**Common Issues & Solutions**:
- Robot doesn't move → Check motor configuration names match code
- Robot spins when trying to go straight → Verify motor directions
- Wheels installed incorrectly → Check wheel handedness and orientation
- Robot drifts when joystick released → Increase deadzone value

**Deliverable**: TeleOp program where drivers can smoothly control robot in all directions

**Time Estimate**: 2 weeks (including practice time)

---

### Phase 3: Refinements and Speed Control (Weeks 3-4)

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
- Driver feedback and iteration

**Resources**:
- **This Guide**: See coding examples in "Coding Approaches" section
- **Game Manual 0 - Driver Control**: [gm0.org - Driver Control](https://gm0.org/en/latest/docs/software/tutorials/teleop.html)
- **Video: "Advanced TeleOp Techniques"**: Search "FTC TeleOp Best Practices"

**Practice Tasks**:
1. Test different response curves and find what feels best
2. Practice precision driving (slow mode) for game piece pickup
3. Practice fast driving (turbo mode) for field traversal
4. Get comfortable switching between modes mid-match
5. Get driver feedback and iterate

**Deliverable**: Polished TeleOp with speed control that drivers prefer

**Time Estimate**: 2 weeks

---

### Phase 4: IMU Integration and Field-Centric (Weeks 5-7)

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
- Why field-centric is a game-changer for omni drives

**Resources**:
- **This Guide**: See "Approach 2: Field-Centric Control" section
- **Mecanum Guide**: See `Mecanum-Drive-Guide.md` "Field-Centric Drive" section
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
6. Compare driver preference: robot-centric vs field-centric

**Common Issues & Solutions**:
- Heading drifts over time → Normal IMU behavior, add reset button
- Wrong directions when robot rotates → Check IMU hub orientation parameters
- Jittery movement → Verify sin/cos math uses radians (not degrees)

**Deliverable**: Field-centric TeleOp that drivers can use confidently in matches

**Time Estimate**: 3 weeks (math concepts may need extra time)

---

### Phase 5: Basic Autonomous (Weeks 8-10)

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
- Leveraging omni-drive advantages (diagonal paths)

**Resources**:
- **Mecanum Guide**: See `Mecanum-Drive-Guide.md` "Autonomous Programming" section
- **FTC SDK Sample**: `RobotAutoDriveByEncoder_Linear.java`
- **Game Manual 0 - Autonomous**: [gm0.org - Autonomous](https://gm0.org/en/latest/docs/software/tutorials/autonomous.html)
- **Video: "FTC Autonomous Programming"**: Search "FTC Autonomous Tutorial"
- **Encoder Explanation**: [Encoders Explained](https://gm0.org/en/latest/docs/software/concepts/encoders.html)

**Practice Tasks**:
1. Drive forward specific distance using encoders
2. Strafe to specific position (leverage omni capability!)
3. Drive diagonally to demonstrate holonomic advantage
4. Create autonomous to park in designated zone
5. Build autonomous sequence: start → score preload → park
6. Test autonomous reliability (run 10 times, measure success rate)

**Calibration Steps**:
1. Measure your motor's encoder counts per revolution (COUNTS_PER_MOTOR_REV)
2. Measure wheel diameter accurately
3. Calculate COUNTS_PER_MM constant
4. Test forward movement over known distance, adjust if needed
5. Test strafing (usually needs 1.1x-1.2x multiplier due to slip)
6. For mecanum: Test diagonal movements

**Common Issues & Solutions**:
- Robot doesn't drive exact distance → Calibrate encoder constants
- Strafing moves at angle → Add strafe multiplier (1.1-1.2x)
- Inconsistent movement → Check battery charge, use RUN_TO_POSITION mode
- Diagonal movements inaccurate → May need different multipliers per direction

**Deliverable**: Working autonomous routine that scores and parks reliably

**Time Estimate**: 3 weeks

---

### Phase 6: Advanced Autonomous (Weeks 11-14) - OPTIONAL

**Goal**: Learn professional-grade autonomous programming

**What to Learn**:
1. Path planning with Road Runner library
2. Dead wheel odometry for accurate localization
3. PID control for precise movement
4. AprilTag detection for position correction
5. Leveraging omni-drive for complex paths

**Key Concepts**:
- Trajectory planning (splines, bezier curves)
- Localization (knowing where robot is on field)
- PID controllers (Proportional-Integral-Derivative)
- Computer vision integration
- State machines for complex autonomous
- Velocity constraints for smooth motion

**Resources**:
- **Road Runner**: [learnroadrunner.com](https://learnroadrunner.com)
- **Road Runner Quickstart**: [GitHub Quickstart](https://github.com/acmerobotics/road-runner-quickstart)
- **Game Manual 0 - Road Runner**: [gm0.org - Road Runner](https://gm0.org/en/latest/docs/software/tutorials/road-runner.html)
- **PID Explained**: [PID Control Simplified](https://www.youtube.com/watch?v=wkfEZmsQqiA)
- **AprilTag Tutorial**: [FTC AprilTag Documentation](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html)
- **FTC ML/Vision**: [FIRST Machine Learning Toolchain](https://ftc-ml.firstinspires.org/)
- **Pedro Pathing**: [Alternative to Road Runner](https://pedropathing.com/)

**Practice Tasks**:
1. Install and configure Road Runner for your drive configuration
2. Tune your robot's drive characteristics
3. Create spline trajectories for smooth paths
4. Implement dead wheel odometry
5. Use AprilTags for position correction mid-autonomous
6. Build complex multi-action autonomous routines

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
  - Excellent drivetrain comparison section
  - Hardware and software tutorials
  - Competition strategies
  - **Must-read sections**:
    - [Drivetrain Types](https://gm0.org/en/latest/docs/hardware/drivetrains.html)
    - [Mecanum Drive Programming](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)
    - [Control System](https://gm0.org/en/latest/docs/software/index.html)

- **FTC Discord Server**: [discord.gg/first-tech-challenge](https://discord.com/invite/first-tech-challenge)
  - Very active community (#programming-help, #hardware-help)
  - Quick answers to questions
  - Event discussions
  - Great for troubleshooting

- **Chief Delphi**: [chiefdelphi.com/tag/ftc](https://www.chiefdelphi.com/tag/ftc)
  - Cross-FRC/FTC technical discussions
  - Build logs from top teams
  - Strategy discussions

### Video Resources
Search YouTube for these topics:
- "FTC Programming Tutorial for Beginners"
- "Mecanum Wheels Explained"
- "Omni Wheels vs Mecanum"
- "FTC Field Centric Drive"
- "How to use RoadRunner FTC"
- "FTC Autonomous Programming"
- "FTC OpMode Tutorial"
- "X-Drive FTC Programming"

**Recommended Channels**:
- FIRST Tech Challenge (official)
- Dexter Industries
- Alan G. Smith (FTC tutorials)
- FTC teams with build logs (search for "FTC reveal" or "FTC build")

### Hardware-Specific Documentation
- **REV Robotics**: [docs.revrobotics.com](https://docs.revrobotics.com/)
  - Control Hub, Expansion Hub documentation
  - Sensor guides (IMU, distance, color)
  - Motor and servo specs
  - Wiring diagrams

- **goBILDA**: [gobildaproducts.com](https://www.gobildaproducts.com/)
  - Motor specifications
  - Mecanum wheel details
  - Omni wheel specifications
  - Build guides and CAD files

- **AndyMark**: [andymark.com](https://www.andymark.com/)
  - Alternative wheel options
  - Motor and component specs

### Programming Tools
- **FTC Dashboard**: [acmerobotics.github.io/ftc-dashboard](https://acmerobotics.github.io/ftc-dashboard/)
  - Real-time telemetry graphing
  - Live configuration editing
  - Essential for Road Runner tuning
  - Field visualization

- **OnBot Java**: Built into Robot Controller
  - Browser-based code editor
  - No Android Studio needed
  - Good for quick tests and field-side edits

- **Android Studio**: Full IDE with debugging
  - Best for serious development
  - Code completion and refactoring
  - Integrated version control

### Competition Preparation
- **Game Manual Part 1**: [firstinspires.org/resource-library/ftc/game-and-season-info](https://www.firstinspires.org/resource-library/ftc/game-and-season-info)
  - Official competition rules
  - Robot construction rules
  - Game-specific regulations

- **Game Manual Part 2**: Technical rules and regulations
  - Robot inspection checklist
  - Safety requirements
  - Field specifications
  - Size and weight limits

### Practice Tools
- **FTC Simulator**: [github.com/Beta8397/virtual_robot](https://github.com/Beta8397/virtual_robot)
  - Test code without physical robot
  - Practice autonomous paths
  - Useful when robot isn't available
  - Supports mecanum and other drive types

---

## Suggested Practice Schedule

### For Teams Meeting 2x Per Week (3-hour sessions)

**Weeks 1-2: Foundation**
- Session 1: Hardware setup, configure Driver Station, verify wheel installation
- Session 2: Run basic samples, understand OpMode structure
- Session 3: Copy robot-centric code for your drive type, test motor directions
- Session 4: Debug and practice driving, verify holonomic motion works

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
- Session 5: Compare robot vs field-centric, choose best for drivers
- Session 6: Final polish for competition

**Weeks 8-10: Autonomous**
- Session 1: Learn encoder basics, time-based movement
- Session 2: Implement encoder-based movement
- Session 3: Calibrate encoder constants for all movement directions
- Session 4: Build autonomous sequence leveraging omni capabilities
- Session 5: Test and refine reliability
- Session 6: Practice autonomous + driver handoff

### For Teams Meeting 3-4x Per Week
- Follow same structure but move faster
- Add extra practice sessions for driver training
- Consider splitting into sub-teams (drive, autonomous, mechanisms, strategy)

---

## Getting Help

### When You're Stuck
1. **Check telemetry**: Add debug output to see what's happening
2. **Test incrementally**: Comment out code until you find the problem
3. **Read error messages**: Android Studio shows helpful compile errors
4. **Verify hardware**: Are motors/wheels configured correctly?
5. **Search GM0**: Someone else likely had same issue
6. **Check this guide**: Review relevant sections
7. **Ask on Discord**: FTC community is very helpful

### Good Questions Get Better Answers
**Bad question**: "My mecanum drive doesn't work, help!"

**Good question**:
> "Using mecanum drive, robot-centric OpMode. When I push left joystick forward, robot spins instead of moving forward. I've verified motor names match hardware config. Motors are: frontLeft (port 0), frontRight (port 1, REVERSED), backLeft (port 2), backRight (port 3, REVERSED). Wheels form X-pattern from top view. Here's my kinematics code: [paste relevant snippet]. What am I doing wrong?"

**Include**:
- What drive type you're using (mecanum, X-drive, etc.)
- What you're trying to do
- What's actually happening (unexpected behavior)
- What you've already tried
- Relevant code snippet
- Hardware setup details (motor ports, wheel configuration)

---

## Milestones and Checkpoints

Use these milestones to track your team's progress:

### ✅ Checkpoint 1: Hardware Verification (Week 1)
- [ ] All wheels are correct type (mecanum, omni, etc.)
- [ ] Wheels installed in correct orientation
- [ ] All 4 motors spin correctly when tested individually
- [ ] Motor ports match hardware configuration
- [ ] Can run basic sample OpMode successfully

### ✅ Checkpoint 2: Basic Movement (Week 2)
- [ ] Robot drives forward in straight line
- [ ] Robot can strafe left/right (key omni-drive test!)
- [ ] Robot can rotate in place
- [ ] Can drive diagonally to demonstrate holonomic motion
- [ ] Driver can control robot smoothly with gamepad

### ✅ Checkpoint 3: Refined TeleOp (Week 4)
- [ ] Speed modes implemented (precision/normal/turbo)
- [ ] Deadzone prevents drift
- [ ] Brake mode enabled for precise stopping
- [ ] Telemetry shows useful debug info
- [ ] Drivers prefer this code over basic version

### ✅ Checkpoint 4: Field-Centric (Week 7)
- [ ] IMU reads heading correctly
- [ ] Field-centric transformation works
- [ ] Reset button implemented
- [ ] Robot moves field-forward regardless of orientation
- [ ] Drivers can use effectively in practice
- [ ] Team decides: robot-centric or field-centric for competition

### ✅ Checkpoint 5: Competition-Ready TeleOp (Week 8)
- [ ] Code is reliable (no crashes in 20+ runs)
- [ ] Drivers are comfortable and confident
- [ ] All controls documented for drivers
- [ ] Backup OpMode available (in case of issues)
- [ ] Can demonstrate omni-drive advantages in practice

### ✅ Checkpoint 6: Basic Autonomous (Week 10)
- [ ] Encoder constants calibrated accurately for all directions
- [ ] Can drive forward/backward to specific distance (±2 inches)
- [ ] Can strafe to specific position (±3 inches)
- [ ] Can drive diagonally accurately
- [ ] Autonomous sequence completes 8/10 times successfully
- [ ] Can score preload and park

### ✅ Checkpoint 7: Competition Autonomous (Week 12)
- [ ] Autonomous is reliable (9/10 success rate)
- [ ] Multiple autonomous options for different starting positions
- [ ] Leverages omni-drive advantages (diagonal/strafe paths)
- [ ] Autonomous integrates with TeleOp (shares hardware config)
- [ ] Tested under competition conditions (low battery, bumped robot, etc.)

---

## Omni-Drive Specific Tips

### Maximizing Your Omni-Drive Advantage

**In TeleOp**:
1. **Strafe to align**: Instead of rotating to face game pieces, strafe while maintaining orientation
2. **Circle-strafe**: Move around obstacles while keeping front/sensors pointed forward
3. **Evasive maneuvers**: Dodge opponents while maintaining offensive position
4. **Quick repositioning**: Diagonal movements are faster than turn-move sequences

**In Autonomous**:
1. **Direct diagonal paths**: Take advantage of holonomic motion for faster routes
2. **Strafe into position**: Approach scoring zones from any angle
3. **Maintain heading**: Keep sensors/cameras pointed at targets while moving
4. **Faster cycles**: Reduce time by eliminating rotation steps

### Common Omni-Drive Mistakes to Avoid

1. **Not using strafe capability**: Teams often forget they can strafe - practice it!
2. **Over-rotating**: Use strafe instead of turning when possible
3. **Ignoring traction loss**: Omni drives slip more - design for it
4. **Poor weight distribution**: Ensure even weight on all wheels
5. **Worn rollers**: Check and replace worn omni/mecanum wheel rollers

---

## Next Steps After This Guide

Once you've mastered omni-drive programming:

1. **Add Mechanisms**: Arm, claw, lift, intake
2. **Vision Processing**: AprilTag detection, color recognition, object tracking
3. **Advanced Autonomous**: Road Runner, path following, trajectory optimization
4. **Multi-Robot Coordination**: Alliance partner communication
5. **Optimization**: Reduce cycle times, improve efficiency
6. **Competition Strategy**: Auto selection, endgame routines, defensive play

### Recommended Advanced Topics
- **Pose estimation**: Combining odometry, IMU, and vision for accurate localization
- **Path planning**: RRT, A*, spline-based trajectories
- **State machines**: Managing complex autonomous sequences
- **Driver assistance**: Auto-alignment, assisted scoring
- **Telemetry analysis**: Recording and analyzing match data

**Remember**: Programming is iterative. Start simple, test often, and improve gradually!

---

*Updated Learning Path for beginner FTC teams - DECODE 2025-2026 season*

---

## Summary

**Omni wheels enable holonomic motion**, giving your robot the ability to move in any direction without rotating first. This requires:

1. **4 motors** for independent control of 3 degrees of freedom
2. **Specialized kinematics** to translate driver input to wheel speeds
3. **Trade-offs** between maneuverability and traction

**Key advantages**:
- Faster autonomous (direct paths)
- Field-centric driving (intuitive control)
- Precise positioning for scoring
- Evasive maneuvers while maintaining orientation

**Coding approaches** range from simple direct control to advanced path planning with Road Runner. Start simple and add complexity as needed.

**Remember**: Great driving is a combination of good hardware, solid code, and practice. Test extensively and iterate!

---

*Document created for FTC Team reference - DECODE 2025-2026 season*
