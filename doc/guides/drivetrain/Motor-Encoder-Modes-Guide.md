# Motor Encoder Modes Guide

## Understanding RUN_WITHOUT_ENCODER vs RUN_USING_ENCODER

This guide explains the key differences between FTC motor control modes and why `RUN_WITHOUT_ENCODER` mode results in faster robot movement than `RUN_USING_ENCODER` mode.

---

## Table of Contents

1. [Overview](#overview)
2. [How Each Mode Works](#how-each-mode-works)
3. [Why Speed Differs](#why-speed-differs)
4. [Performance Comparison](#performance-comparison)
5. [When to Use Each Mode](#when-to-use-each-mode)
6. [Code Examples](#code-examples)
7. [Advanced Techniques](#advanced-techniques)
8. [Troubleshooting](#troubleshooting)
9. [Learning Path](#learning-path)

---

## Overview

FTC motors support four run modes, but the two most commonly used for driving are:

| Mode | Type | Description |
|------|------|-------------|
| `RUN_WITHOUT_ENCODER` | Open-loop | Direct power control - motor receives raw voltage |
| `RUN_USING_ENCODER` | Closed-loop | Velocity-controlled using PID feedback from encoder |

The other two modes are:
- `STOP_AND_RESET_ENCODER` - Resets encoder count to zero
- `RUN_TO_POSITION` - Moves to a specific encoder position

---

## How Each Mode Works

### RUN_WITHOUT_ENCODER (Open-Loop Control)

```
    Power Input (0.0 to 1.0)
           │
           ▼
    ┌──────────────┐
    │   Motor      │ ──────► Wheel Rotation
    │   Driver     │
    └──────────────┘
           │
           ▼
      Direct Voltage
      (% of battery)
```

**How it works:**
1. You set a power value between -1.0 and 1.0
2. The motor controller converts this to a percentage of battery voltage
3. That voltage goes directly to the motor
4. Motor speed depends on voltage, load, and friction

**Key characteristics:**
- **No feedback loop** - the system doesn't know or care how fast the motor is actually spinning
- **Maximum possible speed** - 100% power = 100% available voltage
- **Speed varies** with battery charge level and mechanical load
- **Simplest control** - what you set is what you get

### RUN_USING_ENCODER (Closed-Loop Velocity Control)

```
    Velocity Target
           │
           ▼
    ┌──────────────┐      ┌──────────────┐
    │     PID      │ ───► │    Motor     │ ──────► Wheel Rotation
    │  Controller  │      │    Driver    │              │
    └──────────────┘      └──────────────┘              │
           ▲                                            │
           │              ┌──────────────┐              │
           └───────────── │   Encoder    │ ◄────────────┘
              Feedback    │   (Sensor)   │
                          └──────────────┘
```

**How it works:**
1. You set a power value (interpreted as velocity target)
2. The PID controller calculates required motor voltage
3. Encoder measures actual wheel speed
4. Controller adjusts voltage to match target velocity
5. This loop runs continuously (~50 times per second)

**Key characteristics:**
- **Feedback loop** - constantly adjusts to maintain target speed
- **Consistent velocity** - compensates for battery drain and load changes
- **Speed-limited** - cannot use 100% of motor capability
- **More complex** - requires encoder hardware and tuning

---

## Why Speed Differs

### The Core Reason: Control Headroom

When using `RUN_USING_ENCODER`, the PID controller needs **headroom** to make corrections:

```
RUN_WITHOUT_ENCODER:
├────────────────────────────────────────────────────────┤ 100% Power Available
████████████████████████████████████████████████████████  You can use ALL of it

RUN_USING_ENCODER:
├────────────────────────────────────────────────────────┤ 100% Power Available
██████████████████████████████████████████░░░░░░░░░░░░░░  ~85% usable
                                          │
                                          └─ Reserved for PID corrections
```

### Why Headroom is Necessary

Imagine the PID controller is driving at exactly 100% power to maintain velocity:

1. **Robot hits slight incline** → needs MORE power to maintain speed
2. **Controller is already at 100%** → cannot increase power
3. **Robot slows down** → velocity target missed

By limiting maximum velocity to ~85-90%, the controller always has reserve power to:
- Speed up when encountering resistance
- Compensate for one motor being slightly weaker
- Handle battery voltage fluctuations

### Additional Factors

| Factor | Impact on Speed |
|--------|-----------------|
| **Conservative SDK limits** | Default max velocity is set safely low |
| **PID tuning overhead** | Needs margin for stable control |
| **Motor characterization** | SDK uses conservative motor specs |
| **Safety margins** | Prevents motor damage from overcurrent |

---

## Performance Comparison

### Speed Test Results (Typical)

| Mode | Relative Speed | Consistency | Battery Sensitivity |
|------|----------------|-------------|---------------------|
| `RUN_WITHOUT_ENCODER` | 100% | Variable | High |
| `RUN_USING_ENCODER` | 85-90% | Excellent | Low |

### Detailed Comparison

| Aspect | RUN_WITHOUT_ENCODER | RUN_USING_ENCODER |
|--------|---------------------|-------------------|
| **Maximum Speed** | Highest possible | ~85-90% of max |
| **Speed Consistency** | Varies with conditions | Very consistent |
| **Battery Impact** | Slows as battery drains | Compensates automatically |
| **Left/Right Matching** | May drift | Stays synchronized |
| **Response Time** | Instant | Slight lag (PID settling) |
| **Precision** | Lower | Higher |
| **CPU Usage** | Minimal | Slightly higher |
| **Encoder Required** | No (can still read) | Yes |

### Real-World Example

**Fresh battery (13.0V) vs. Used battery (12.0V):**

```
RUN_WITHOUT_ENCODER:
  Fresh: ████████████████████████████████ 100% speed
  Used:  ███████████████████████████░░░░░  92% speed (slower!)

RUN_USING_ENCODER:
  Fresh: ██████████████████████████░░░░░░  85% speed
  Used:  ██████████████████████████░░░░░░  85% speed (same!)
```

---

## When to Use Each Mode

### Use RUN_WITHOUT_ENCODER When:

- **Maximum speed is critical** (TeleOp driving, end-game sprint)
- **You don't have encoders** on the motors
- **Simplicity matters** (quick prototyping)
- **Driver controls speed** (human feedback replaces sensor feedback)
- **Short autonomous movements** where consistency isn't critical

```java
// TeleOp driving - maximum responsiveness
@TeleOp(name = "Fast Drive")
public class FastDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Use raw power mode for maximum speed
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            leftDrive.setPower(drive + turn);
            rightDrive.setPower(drive - turn);
        }
    }
}
```

### Use RUN_USING_ENCODER When:

- **Consistency matters** (autonomous routines)
- **Precision is required** (alignment, positioning)
- **Both sides must match** (driving straight)
- **Long matches** where battery drain is significant
- **Repeatable movements** (same speed every time)

```java
// Autonomous - consistent, repeatable movement
@Autonomous(name = "Consistent Auto")
public class ConsistentAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Use encoder mode for consistent velocity
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Drive forward at 50% velocity - will be the same every time
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(2000);

        // Stop
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
```

---

## Code Examples

### Example 1: Hybrid Approach - Fast Driving with Distance Tracking

Get maximum speed while still tracking distance traveled:

```java
@TeleOp(name = "Hybrid Drive")
public class HybridDrive extends LinearOpMode {

    static final double COUNTS_PER_INCH = 537.7 / (4.0 * Math.PI); // Example for 4" wheel

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Reset encoders first
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Switch to raw power mode (encoders still work!)
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Full speed driving
            leftDrive.setPower(-gamepad1.left_stick_y);
            rightDrive.setPower(-gamepad1.left_stick_y);

            // Calculate distance traveled (encoders still readable!)
            double leftInches = leftDrive.getCurrentPosition() / COUNTS_PER_INCH;
            double rightInches = rightDrive.getCurrentPosition() / COUNTS_PER_INCH;
            double avgInches = (leftInches + rightInches) / 2.0;

            telemetry.addData("Distance", "%.1f inches", avgInches);
            telemetry.update();
        }
    }
}
```

### Example 2: Mode Switching Based on Situation

Switch modes dynamically based on what you're doing:

```java
@TeleOp(name = "Smart Drive")
public class SmartDrive extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Start in fast mode
        setFastMode();

        waitForStart();

        while (opModeIsActive()) {
            // Press A for precision mode, B for fast mode
            if (gamepad1.a) {
                setPrecisionMode();
            } else if (gamepad1.b) {
                setFastMode();
            }

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            leftDrive.setPower(drive + turn);
            rightDrive.setPower(drive - turn);

            telemetry.addData("Mode", leftDrive.getMode());
            telemetry.update();
        }
    }

    private void setFastMode() {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPrecisionMode() {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
```

### Example 3: Using DcMotorEx for Velocity Control

For more control over `RUN_USING_ENCODER` mode, use `DcMotorEx`:

```java
@TeleOp(name = "Velocity Control")
public class VelocityControl extends LinearOpMode {

    @Override
    public void runOpMode() {
        // DcMotorEx provides velocity control methods
        DcMotorEx leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        DcMotorEx rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Set velocity directly in ticks per second
            // This gives more precise control than setPower()
            double targetVelocity = -gamepad1.left_stick_y * 2000; // Max ~2000 ticks/sec

            leftDrive.setVelocity(targetVelocity);
            rightDrive.setVelocity(targetVelocity);

            // Read actual velocity
            telemetry.addData("Target", "%.0f ticks/sec", targetVelocity);
            telemetry.addData("Actual L", "%.0f ticks/sec", leftDrive.getVelocity());
            telemetry.addData("Actual R", "%.0f ticks/sec", rightDrive.getVelocity());
            telemetry.update();
        }
    }
}
```

---

## Advanced Techniques

### Technique 1: Custom PID Tuning

If `RUN_USING_ENCODER` is too slow, you can tune the PID coefficients:

```java
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

// Get current PID coefficients
PIDFCoefficients pidOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

// Create new coefficients (increase P for more aggressive response)
PIDFCoefficients pidNew = new PIDFCoefficients(
    pidOrig.p * 1.2,  // Proportional - increase for faster response
    pidOrig.i,        // Integral - usually leave alone
    pidOrig.d,        // Derivative - damping
    pidOrig.f         // Feedforward - base power
);

// Apply new coefficients
motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
```

> **Warning:** Aggressive PID tuning can cause oscillation or motor damage. Test carefully!

### Technique 2: Implement Your Own Velocity Control

For maximum flexibility, implement custom velocity control:

```java
public class CustomVelocityController {
    private DcMotorEx motor;
    private double targetVelocity = 0;
    private double kP = 0.001;  // Tune these values
    private double kF = 0.0005;

    public CustomVelocityController(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void update() {
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        // Simple P + Feedforward control
        double power = (error * kP) + (targetVelocity * kF);
        power = Math.max(-1.0, Math.min(1.0, power));  // Clamp to [-1, 1]

        motor.setPower(power);
    }
}
```

### Technique 3: Measure Maximum Velocity

Find your motor's actual maximum velocity for better scaling:

```java
@TeleOp(name = "Velocity Test")
public class VelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "test_motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double maxVelocity = 0;

        waitForStart();

        // Run at full power
        motor.setPower(1.0);

        while (opModeIsActive()) {
            double velocity = motor.getVelocity();
            if (velocity > maxVelocity) {
                maxVelocity = velocity;
            }

            telemetry.addData("Current Velocity", "%.0f ticks/sec", velocity);
            telemetry.addData("Max Velocity", "%.0f ticks/sec", maxVelocity);
            telemetry.addData("Instructions", "Let motor reach full speed, note max value");
            telemetry.update();
        }

        motor.setPower(0);
    }
}
```

---

## Troubleshooting

### Problem: Robot drives slower than expected in RUN_USING_ENCODER

**Solutions:**
1. Verify encoders are connected and working (`getCurrentPosition()` should change)
2. Check encoder direction matches motor direction
3. Try increasing power value (0.8 or 0.9 instead of 0.5)
4. Consider using `DcMotorEx.setVelocity()` for direct velocity control

### Problem: Robot drifts to one side in RUN_WITHOUT_ENCODER

**Solutions:**
1. Motors may have different characteristics - this is normal
2. Mechanically adjust (check wheel alignment, friction)
3. Software compensation: multiply weaker side by correction factor
4. Switch to `RUN_USING_ENCODER` for automatic compensation

### Problem: Encoders read zero or don't change

**Solutions:**
1. Verify encoder cables are connected to correct ports
2. Check that encoder is mechanically coupled to motor shaft
3. Ensure you haven't called `STOP_AND_RESET_ENCODER` without switching modes
4. Verify motor configuration in Driver Station matches code

### Problem: Jerky movement in RUN_USING_ENCODER

**Solutions:**
1. Add ramping/acceleration control to smooth inputs
2. Reduce PID gains if you've modified them
3. Ensure encoder cables aren't loose (intermittent signal)
4. Check for mechanical binding or friction

---

## Summary: Quick Reference

```
┌─────────────────────────────────────────────────────────────────┐
│                    MOTOR MODE DECISION TREE                     │
└─────────────────────────────────────────────────────────────────┘

Need maximum speed?
├── YES → RUN_WITHOUT_ENCODER
│         • Direct power control
│         • 100% motor capability
│         • Speed varies with battery
│
└── NO → Need consistent speed?
         ├── YES → RUN_USING_ENCODER
         │         • PID velocity control
         │         • ~85-90% max speed
         │         • Battery-independent
         │
         └── NO → Need to go to a position?
                  └── YES → RUN_TO_POSITION
                            • Position-based control
                            • Automatic stop at target
```

### Key Takeaways

1. **RUN_WITHOUT_ENCODER is faster** because it uses 100% of available motor power
2. **RUN_USING_ENCODER is consistent** because PID control compensates for variables
3. **You can read encoders in any mode** - mode only affects how power is applied
4. **Choose based on your needs** - speed vs. consistency trade-off
5. **Hybrid approaches work** - switch modes or use custom control when needed

---

## Learning Path

### Beginner Resources
- [FTC Motor Control Basics](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/motors/motors.html) - Official FTC documentation on motors
- [FIRST Tech Challenge SDK Javadoc - DcMotor](https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/DcMotor.html) - API reference

### Intermediate Resources
- [Game Manual 0 - Motors](https://gm0.org/en/latest/docs/software/concepts/control-loops.html) - Community guide on control loops
- [FTC SDK Sample: RobotHardware](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotHardware.java) - Example hardware class

### Advanced Resources
- [PID Control Theory](https://gm0.org/en/latest/docs/software/concepts/pid-controllers.html) - Understanding PID controllers
- [Road Runner](https://learnroadrunner.com/) - Advanced motion profiling library
- [FTC Lib](https://docs.ftclib.org/) - Community library with enhanced motor control

### Video Tutorials
- Search YouTube for "FTC encoder tutorial" for visual explanations
- FTC official channel has competition-relevant tutorials

---

*Document created for DECODE 2025-2026 season*
*Last updated: December 2024*
