# BasicOpMode_Linear vs StarterBotTeleop: Complete Comparison Guide

**Last Updated:** 2025-11-10
**FTC SDK Version:** 11.0 (DECODE Season 2025-2026)

## Overview

This document provides a comprehensive comparison between two fundamental FTC OpMode examples: `BasicOpMode_Linear` (a minimal teaching example) and `StarterBotTeleop` (a production-ready competition robot). Understanding these differences will help teams choose the right starting point and learn key programming concepts.

---

## Quick Reference Table

| Feature | BasicOpMode_Linear | StarterBotTeleop |
|---------|-------------------|------------------|
| **OpMode Type** | LinearOpMode (sequential) | OpMode (iterative) |
| **Complexity** | Beginner skeleton | Competition-ready |
| **Hardware** | 2 drive motors | 2 drive motors + launcher + 2 servos |
| **Motor Control** | Power-based (simple) | Velocity-based with PIDF |
| **State Management** | None | State machine for launcher |
| **Code Structure** | Inline control | Separate functions |
| **Learning Focus** | OpMode basics | Advanced concepts |
| **Ready to Use** | Teaching template | Production code |

---

## Detailed Comparison

### 1. OpMode Architecture

#### BasicOpMode_Linear
- **Type:** Extends `LinearOpMode`
- **Structure:** Sequential, blocking execution model
- **Control Flow:** Top-to-bottom with `while (opModeIsActive())` loop
- **Best For:**
  - Beginners learning OpMode structure
  - Autonomous routines with sequential steps
  - Simple teleop programs

```java
@Override
public void runOpMode() {
    // Initialize hardware
    waitForStart();

    // Main loop runs sequentially
    while (opModeIsActive()) {
        // Your code here executes in order
    }
}
```

**File Location:** `BasicOpMode_Linear.java:62-114`

#### StarterBotTeleop
- **Type:** Extends `OpMode`
- **Structure:** Iterative, state-machine model
- **Control Flow:** Separate lifecycle methods called repeatedly by framework
- **Best For:**
  - Complex teleop programs with multiple subsystems
  - Programs requiring parallel operations
  - Production competition code

```java
@Override
public void init() {
    // Called once when INIT is pressed
}

@Override
public void loop() {
    // Called repeatedly (~50Hz) during teleop
}
```

**File Location:** `StarterBotTeleop.java:128-243`

---

### 2. Hardware Components

#### BasicOpMode_Linear
**Hardware Count:** 2 devices
- `leftDrive` - DcMotor
- `rightDrive` - DcMotor

**Configuration:**
```java
leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
```

**File Location:** `BasicOpMode_Linear.java:59-60, 70-71`

#### StarterBotTeleop
**Hardware Count:** 5 devices
- `leftDrive` - DcMotor (drivetrain)
- `rightDrive` - DcMotor (drivetrain)
- `launcher` - DcMotorEx (flywheel shooter)
- `leftFeeder` - CRServo (ball feeder)
- `rightFeeder` - CRServo (ball feeder)

**Configuration:**
```java
leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
launcher = hardwareMap.get(DcMotorEx.class, "launcher");
leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
```

**File Location:** `StarterBotTeleop.java:88-92, 137-141`

**Notes:**
- Uses `DcMotorEx` for advanced motor features (velocity control)
- Continuous rotation servos for game element manipulation

---

### 3. Motor Control Methods

#### BasicOpMode_Linear: Simple Power Control
**Mode:** `RUN_WITHOUT_ENCODER` (default)
**Control Method:** Direct power setting (-1.0 to 1.0)
**Characteristics:**
- Simple to understand and use
- No feedback control
- Speed varies with battery voltage and load
- Good for basic driving

```java
leftDrive.setPower(leftPower);   // Direct power control
rightDrive.setPower(rightPower);
```

**File Location:** `BasicOpMode_Linear.java:106-107`

#### StarterBotTeleop: Advanced Velocity Control
**Mode:** `RUN_USING_ENCODER`
**Control Method:** PIDF closed-loop velocity control
**Characteristics:**
- Maintains constant speed regardless of load
- Uses encoder feedback
- Requires tuned PIDF coefficients
- Essential for launcher consistency

```java
// Set motor mode for velocity control
launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Custom PIDF coefficients for launcher
launcher.setPIDFCoefficients(
    DcMotor.RunMode.RUN_USING_ENCODER,
    new PIDFCoefficients(300, 0, 0, 10)
);

// Set target velocity (not power!)
launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
```

**File Location:** `StarterBotTeleop.java:160, 177, 226, 271`

**PIDF Values Explained:**
- **P = 300:** Strong proportional response to velocity errors
- **I = 0:** No integral correction (not needed for flywheels)
- **D = 0:** No derivative dampening
- **F = 10:** Feedforward baseline power to overcome friction

---

### 4. Drive Control Implementation

#### BasicOpMode_Linear: Inline POV/Arcade Drive
**Style:** Inline calculations in main loop
**Features:**
- POV (Point of View) mode by default
- Tank mode available (commented out)
- Power clipping to valid range

```java
// POV Mode (default)
double drive = -gamepad1.left_stick_y;
double turn  =  gamepad1.right_stick_x;
leftPower    = Range.clip(drive + turn, -1.0, 1.0);
rightPower   = Range.clip(drive - turn, -1.0, 1.0);

// Tank Mode (alternative - commented out)
// leftPower  = -gamepad1.left_stick_y;
// rightPower = -gamepad1.right_stick_y;

leftDrive.setPower(leftPower);
rightDrive.setPower(rightPower);
```

**File Location:** `BasicOpMode_Linear.java:90-107`

**Controls:**
- Left stick Y-axis: Forward/backward
- Right stick X-axis: Turning

#### StarterBotTeleop: Function-Based Arcade Drive
**Style:** Separate `arcadeDrive()` function
**Features:**
- Cleaner, more maintainable code
- Easier to modify or swap drive systems
- No power clipping (relies on motor controller limits)

```java
// In main loop - clean function call
arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

// Function implementation
void arcadeDrive(double forward, double rotate) {
    leftPower = forward + rotate;
    rightPower = forward - rotate;

    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
}
```

**File Location:** `StarterBotTeleop.java:219, 252-261`

**Controls:**
- Left stick Y-axis: Forward/backward
- Right stick X-axis: Turning

---

### 5. State Machine Pattern (StarterBotTeleop Only)

#### Concept
A **state machine** allows complex, time-based operations to run without blocking the main loop. Instead of using `sleep()` or nested loops, the code tracks its current "state" and transitions between states based on conditions.

#### LaunchState Enum
```java
private enum LaunchState {
    IDLE,        // Waiting for shot request
    SPIN_UP,     // Accelerating launcher to target speed
    LAUNCH,      // Starting feeder servos
    LAUNCHING,   // Feeding projectile (timed operation)
}
```

**File Location:** `StarterBotTeleop.java:112-117`

#### State Machine Flow
```
IDLE → [User presses bumper] → SPIN_UP
SPIN_UP → [Velocity > threshold] → LAUNCH
LAUNCH → [Start timer, run feeders] → LAUNCHING
LAUNCHING → [Timer expires] → IDLE
```

#### Implementation
```java
void launch(boolean shotRequested) {
    switch (launchState) {
        case IDLE:
            if (shotRequested) {
                launchState = LaunchState.SPIN_UP;
            }
            break;

        case SPIN_UP:
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                launchState = LaunchState.LAUNCH;
            }
            break;

        case LAUNCH:
            leftFeeder.setPower(FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);
            feederTimer.reset();
            launchState = LaunchState.LAUNCHING;
            break;

        case LAUNCHING:
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                launchState = LaunchState.IDLE;
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }
            break;
    }
}
```

**File Location:** `StarterBotTeleop.java:263-290`

#### Why State Machines?
- **Non-blocking:** Main loop continues running
- **Parallel operations:** Drive while launcher spins up
- **Clear logic:** Easy to understand and debug
- **Scalable:** Easy to add new states or subsystems

---

### 6. Motor Configuration Features

#### BasicOpMode_Linear: Basic Setup
```java
// Motor direction only
leftDrive.setDirection(DcMotor.Direction.REVERSE);
rightDrive.setDirection(DcMotor.Direction.FORWARD);
```

**File Location:** `BasicOpMode_Linear.java:76-77`

#### StarterBotTeleop: Advanced Configuration
```java
// Motor directions
leftDrive.setDirection(DcMotor.Direction.REVERSE);
rightDrive.setDirection(DcMotor.Direction.FORWARD);

// Encoder mode for velocity control
launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Zero power behavior (brake vs coast)
leftDrive.setZeroPowerBehavior(BRAKE);
rightDrive.setZeroPowerBehavior(BRAKE);
launcher.setZeroPowerBehavior(BRAKE);

// Custom PIDF coefficients
launcher.setPIDFCoefficients(
    DcMotor.RunMode.RUN_USING_ENCODER,
    new PIDFCoefficients(300, 0, 0, 10)
);

// Servo directions
leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
```

**File Location:** `StarterBotTeleop.java:150-151, 160, 167-169, 177, 183`

**ZeroPowerBehavior.BRAKE vs COAST:**
- **BRAKE:** Motors resist movement when power is 0 (more controllable)
- **COAST:** Motors spin freely when power is 0 (less control)

---

### 7. Package Location & Status

#### BasicOpMode_Linear
**Original Package:** `org.firstinspires.ftc.robotcontroller.external.samples`
**Updated Package:** `org.firstinspires.ftc.teamcode` *(after enabling)*
**Status:** Originally `@Disabled` - now enabled and ready to use

**File Location:** `BasicOpMode_Linear.java:30, 54`

#### StarterBotTeleop
**Package:** `org.firstinspires.ftc.teamcode`
**Status:** `@Disabled` commented out - production ready

**File Location:** `StarterBotTeleop.java:33, 72`

---

### 8. OpMode Annotations

#### BasicOpMode_Linear
```java
@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {
```

**File Location:** `BasicOpMode_Linear.java:53-54`

#### StarterBotTeleop
```java
@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
```

**File Location:** `StarterBotTeleop.java:71-73`

---

## Control Theory: PID vs PIDF

### PID Control (Not Used in StarterBotTeleop)
**PID** = Proportional-Integral-Derivative

A purely reactive feedback control system:
1. **P (Proportional):** Reacts to current error
   - Formula: `correction = Kp × error`
   - Higher P = stronger response, but can overshoot

2. **I (Integral):** Corrects accumulated past errors
   - Formula: `correction = Ki × ∫error dt`
   - Eliminates steady-state error
   - Too high = instability

3. **D (Derivative):** Dampens oscillations
   - Formula: `correction = Kd × (d error/dt)`
   - Smooths response, reduces overshoot
   - Too high = sluggish response

### PIDF Control (Used in StarterBotTeleop)
**PIDF** = Proportional-Integral-Derivative-Feedforward

Adds proactive control:
4. **F (Feedforward):** Predicts required power based on target
   - Formula: `power = Kf × target_velocity`
   - Provides baseline power independent of error
   - Especially useful for systems with known dynamics (friction, gravity)
   - Reduces reliance on feedback, improves response time

### Why PIDF for Launcher Wheels?

**Problem with PID alone:**
- Launcher wheels need constant power to overcome friction
- PID waits for error to accumulate before responding
- Results in slow acceleration and velocity fluctuations

**PIDF Solution:**
```java
// P=300, I=0, D=0, F=10
launcher.setPIDFCoefficients(
    DcMotor.RunMode.RUN_USING_ENCODER,
    new PIDFCoefficients(300, 0, 0, 10)
);
```

**File Location:** `StarterBotTeleop.java:177`

**How it works:**
1. **F term (10):** Immediately applies baseline power when target velocity is set
2. **P term (300):** Aggressively corrects any velocity errors
3. **I term (0):** Not needed - flywheels don't have steady-state error
4. **D term (0):** Not needed - system is already well-damped

**Result:**
- Fast spin-up to target velocity
- Consistent velocity under varying loads
- No overshoot or oscillation

---

## Telemetry Comparison

#### BasicOpMode_Linear
```java
telemetry.addData("Status", "Run Time: " + runtime.toString());
telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
telemetry.update();
```

**File Location:** `BasicOpMode_Linear.java:110-112`

**Displays:**
- Elapsed runtime
- Left/right motor powers

#### StarterBotTeleop
```java
telemetry.addData("State", launchState);
telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
telemetry.addData("motorSpeed", launcher.getVelocity());
```

**File Location:** `StarterBotTeleop.java:239-241`

**Displays:**
- Current launcher state (IDLE, SPIN_UP, etc.)
- Left/right drive motor powers
- Actual launcher velocity (encoder feedback)

---

## When to Use Each OpMode

### Use BasicOpMode_Linear When:
✅ Learning FTC programming fundamentals
✅ Building simple autonomous routines
✅ Creating basic teleop for testing
✅ Teaching new team members
✅ Prototyping drive systems
✅ You need sequential, step-by-step execution

### Use StarterBotTeleop When:
✅ Building competition robots
✅ Implementing complex subsystems
✅ Need velocity-controlled mechanisms
✅ Managing multiple parallel operations
✅ Implementing game element manipulation
✅ Learning advanced programming patterns

### Hybrid Approach
Many teams start with BasicOpMode_Linear concepts and gradually add StarterBotTeleop features:
1. Start with basic drive (BasicOpMode_Linear)
2. Add brake mode and better motor configuration
3. Implement subsystems with separate functions
4. Add state machines for complex operations
5. Implement velocity control for launchers/intakes

---

## Key Programming Concepts Demonstrated

### BasicOpMode_Linear Teaches:
- ✅ LinearOpMode structure
- ✅ Hardware initialization
- ✅ waitForStart() pattern
- ✅ Main loop with opModeIsActive()
- ✅ Gamepad input reading
- ✅ Basic math for drive control
- ✅ Telemetry basics
- ✅ Motor direction configuration

### StarterBotTeleop Teaches:
- ✅ OpMode (iterative) structure
- ✅ State machine pattern
- ✅ Function-based code organization
- ✅ DcMotorEx advanced features
- ✅ Velocity control with encoders
- ✅ PIDF tuning
- ✅ ZeroPowerBehavior configuration
- ✅ Continuous rotation servos
- ✅ Timer-based operations (ElapsedTime)
- ✅ Multi-subsystem coordination

---

## Mecanum Wheel Configuration Note

**Important:** StarterBotTeleop is designed for the goBILDA StarterBot which uses:
- 2 motors (left_drive, right_drive)
- 104mm mecanum wheels

**With this 2-motor configuration:**
- ✅ Forward/backward movement works normally
- ✅ Rotation works normally
- ❌ Strafing is NOT available (requires 4 motors)

The mecanum wheels in a 2-motor setup function like regular wheels. For full omnidirectional movement (strafing), you need a 4-motor mecanum drivetrain. See `doc/Mecanum-Drive-Guide.md` for details.

**File Reference:** `StarterBotTeleop.java:52-60`

---

## Hardware Configuration Requirements

### BasicOpMode_Linear
Configure in Driver Station app:
- Motor: `left_drive`
- Motor: `right_drive`

### StarterBotTeleop
Configure in Driver Station app:
- Motor: `left_drive`
- Motor: `right_drive`
- Motor: `launcher` (must support encoders)
- Continuous Rotation Servo: `left_feeder`
- Continuous Rotation Servo: `right_feeder`

**Note:** Hardware names in configuration MUST match the strings in `hardwareMap.get()` calls.

---

## Learning Path for Beginner Teams

### Week 1-2: BasicOpMode_Linear
1. Study the code structure
2. Deploy to robot and test drive
3. Experiment with Tank mode vs POV mode
4. Modify telemetry to add custom data
5. Adjust motor directions for your robot

### Week 3-4: Intermediate Concepts
1. Add brake mode to BasicOpMode_Linear
2. Create separate functions for drive control
3. Add button controls for additional features
4. Implement simple autonomous routines

### Week 5-6: StarterBotTeleop Concepts
1. Study the state machine pattern
2. Understand PIDF control theory
3. Implement a simple state machine in your code
4. Add velocity-controlled subsystems

### Week 7+: Competition Development
1. Combine concepts from both OpModes
2. Design subsystems with state machines
3. Tune PIDF coefficients for mechanisms
4. Build autonomous routines with LinearOpMode
5. Build teleop with iterative OpMode

---

## Common Pitfalls & Solutions

### BasicOpMode_Linear
❌ **Problem:** Robot doesn't drive straight
✅ **Solution:** Check motor directions (lines 76-77), may need to reverse

❌ **Problem:** Controls feel inverted
✅ **Solution:** Adjust negative signs on gamepad input (lines 95-96)

❌ **Problem:** Robot too fast/sensitive
✅ **Solution:** Multiply power values by 0.5 or 0.7 to slow down

### StarterBotTeleop
❌ **Problem:** Launcher velocity unstable
✅ **Solution:** Tune PIDF coefficients (line 177), increase F term

❌ **Problem:** State machine gets stuck
✅ **Solution:** Add telemetry to show current state, check transition conditions

❌ **Problem:** Encoders not working
✅ **Solution:** Verify encoder cables connected to motor controller

---

## Additional Resources

### Official Documentation
- **FTC Documentation:** https://ftc-docs.firstinspires.org/
- **Javadoc Reference:** https://javadoc.io/doc/org.firstinspires.ftc
- **Control Systems Intro:** https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/control_system_intro/control-system-intro.html

### Community Resources
- **Game Manual 0 (GM0):** https://gm0.org/en/latest/
- **PID Tutorial:** https://gm0.org/en/latest/docs/software/concepts/control-loops.html
- **FTC Community Forum:** https://ftc-community.firstinspires.org/

### Repository Documentation
- `doc/OpMode-Types-Guide.md` - Detailed guide on LinearOpMode vs OpMode
- `doc/Mecanum-Drive-Guide.md` - Mecanum wheel configurations
- `doc/Advanced-Patterns-From-BunyipsFTC.md` - Advanced code patterns
- `doc/Advanced-Patterns-From-Brighton-FTC.md` - Competition-proven patterns

### Using DeepWiki MCP Tool
For deeper insights into the FTC SDK, use:
```
Ask DeepWiki about FIRST-Tech-Challenge/FtcRobotController
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-11-10 | Initial documentation |

---

## File References

- **BasicOpMode_Linear:** `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BasicOpMode_Linear.java`
- **StarterBotTeleop:** `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/StarterBotTeleop.java`

---

**Document maintained by:** FTC SDK Documentation Team
**For questions or corrections:** Refer to your team mentor or FTC community forums
