# Advanced FTC Patterns: Learning from BunyipsFTC

**Source:** Murray Bridge Bunyips FTC Team (15215, 22407, 24736)
**Library:** [BunyipsLib](https://github.com/Murray-Bridge-Bunyips/BunyipsLib)
**Language:** Kotlin + Java hybrid approach
**Last Updated:** 2025-01-09

---

## Table of Contents

1. [Overview](#overview)
2. [Key Architectural Patterns](#key-architectural-patterns)
3. [Command-Based OpMode](#command-based-opmode)
4. [Hardware Abstraction](#hardware-abstraction)
5. [Task-Based Programming](#task-based-programming)
6. [Advanced Features](#advanced-features)
7. [What We Can Learn](#what-we-can-learn)
8. [Implementation Examples](#implementation-examples)
9. [Comparison with Standard FTC](#comparison-with-standard-ftc)
10. [Recommendations](#recommendations)

---

## Overview

BunyipsFTC is a highly advanced FTC framework developed by Australian teams that showcases professional-level software architecture patterns. Their custom library (BunyipsLib) demonstrates several advanced concepts we can learn from:

### Key Innovations

1. **Command-Based Architecture** - Inspired by WPILib (FRC)
2. **Task System** - Composable, reusable robot actions
3. **Kotlin-First Design** - Modern language features
4. **Hardware Abstraction** - Clean separation of concerns
5. **Subsystem Architecture** - Modular robot components
6. **Declarative Programming** - Less boilerplate, more clarity

### Their Competition Record
- **4+ robots** actively competing (2024-2025 season)
- **Professional-grade codebase** with modular design
- **Extensive use** of RoadRunner for autonomous
- **Advanced vision processing** and localization

---

## Key Architectural Patterns

### 1. Command-Based OpMode

BunyipsFTC uses a **command-based architecture** similar to FIRST Robotics Competition (FRC):

```kotlin
@TeleOp(name = "TeleOp")
open class MainTeleOp : CommandBasedBunyipsOpMode() {
    override fun assignCommands() {
        // Set default drive task
        HolonomicVectorDriveTask(gamepad1, Proto.drive).setAsDefaultTask()

        // Button mappings with inline tasks
        driver() whenPressed Controls.BACK run
            HolonomicDriveTask(gamepad1, Proto.drive)
            finishIf { gamepad1 rising Controls.BACK }

        // Subsystem control with lambda
        Proto.lift.tasks.control { -gamepad2.lsy.toDouble() }.setAsDefaultTask()

        // Delta control for rotators
        Proto.rotator.tasks.controlDelta {
            gamepad2.rsy.toDouble() * (timer.deltaTime() to Seconds)
        }.setAsDefaultTask()

        // Conditional control
        Proto.intake.tasks.control {
            if (gamepad2.x) 1.0
            else if (gamepad2.y) -1.0
            else 0.0
        }.setAsDefaultTask()

        // Complex button conditions
        operator() whenRising (Controls.Analog.RIGHT_TRIGGER to { v -> v == 1.0f })
            run Proto.lift.tasks.home()
            finishIf { gamepad2.lsy != 0.0f }
    }
}
```

**Key Concepts:**
- **Tasks** - Reusable robot actions (like Commands in FRC)
- **Default Tasks** - Run when no other task is active
- **Button Bindings** - Declarative control mapping
- **Finish Conditions** - Tasks automatically end when condition met
- **Lambda Expressions** - Inline control logic

### 2. Robot Configuration as Object

All hardware is defined in a **single configuration object**:

```kotlin
@RobotConfig.AutoInit
object Proto : RobotConfig() {
    val hw = Hardware()  // Raw hardware

    // High-level subsystems
    lateinit var drive: MecanumDrive
    lateinit var rotator: Switch
    lateinit var lift: HoldableActuator
    lateinit var intake: Actuator

    override fun onRuntime() {
        // Hardware initialization with builders
        hw.imu = getHardware("imu", IMUEx::class.java) {
            it.lazyInitialize(
                IMU.Parameters(
                    RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
                )
            )
        }

        // Motor configuration with settings
        hw.fl = getHardware("fl", Motor::class.java) {
            it.direction = DcMotorSimple.Direction.FORWARD
            it.setPowerDeltaThreshold(0.02)  // Deadband
        }

        // ... more hardware setup
    }
}
```

**Benefits:**
- ✅ **Single source of truth** for robot configuration
- ✅ **Type-safe** hardware access
- ✅ **Builder pattern** for initialization
- ✅ **Automatic error handling** in framework
- ✅ **Shared across OpModes** - configure once, use everywhere

### 3. Subsystem Architecture

BunyipsFTC uses **subsystem classes** to encapsulate robot components:

```kotlin
// Example subsystem types:
lateinit var drive: MecanumDrive         // Drivetrain
lateinit var lift: HoldableActuator      // Lift with position hold
lateinit var rotator: Switch             // Binary position mechanism
lateinit var intake: Actuator            // Continuous actuator
```

Each subsystem:
- Encapsulates hardware and logic
- Provides task methods for control
- Manages its own state
- Independent from other subsystems

### 4. Task-Based Programming

**Tasks** are composable units of robot behavior:

```kotlin
// Basic control task
Proto.lift.tasks.control { -gamepad2.lsy.toDouble() }

// Position control task
Proto.lift.tasks.home()

// Delta control task (for time-based changes)
Proto.rotator.tasks.controlDelta {
    gamepad2.rsy.toDouble() * (timer.deltaTime() to Seconds)
}

// Conditional task
Proto.intake.tasks.control {
    if (gamepad2.x) 1.0 else if (gamepad2.y) -1.0 else 0.0
}
```

Tasks can be:
- **Chained** - Run sequentially
- **Parallel** - Run simultaneously
- **Conditional** - Based on sensors/state
- **Timed** - Automatic timeout
- **Interruptible** - Cancel when needed

---

## Command-Based OpMode

### Traditional OpMode (Standard FTC)

```java
@TeleOp(name="Traditional")
public class Traditional extends OpMode {
    private DcMotor lift;
    private DcMotor intake;

    @Override
    public void loop() {
        // Manual state management
        if (gamepad1.a) {
            lift.setPower(1.0);
        } else if (gamepad1.b) {
            lift.setPower(-1.0);
        } else {
            lift.setPower(0);
        }

        // More state management
        if (gamepad1.x) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }

        // ... lots of if statements
    }
}
```

### Command-Based (BunyipsFTC)

```kotlin
@TeleOp(name="Command-Based")
class CommandBased : CommandBasedBunyipsOpMode() {
    override fun assignCommands() {
        // Declarative control - no state management!
        Proto.lift.tasks.control {
            -gamepad2.lsy.toDouble()
        }.setAsDefaultTask()

        Proto.intake.tasks.control {
            if (gamepad2.x) 1.0 else 0.0
        }.setAsDefaultTask()

        // That's it! Framework handles everything else
    }
}
```

**Advantages:**
- ✅ No manual state tracking
- ✅ Cleaner, more readable code
- ✅ Automatic task scheduling
- ✅ Built-in command composition
- ✅ Easier to test and debug

---

## Hardware Abstraction

### Enhanced Motor Class

BunyipsFTC wraps motors with enhanced functionality:

```kotlin
hw.fl = getHardware("fl", Motor::class.java) {
    it.direction = DcMotorSimple.Direction.FORWARD
    it.setPowerDeltaThreshold(0.02)  // Only update if change > 2%
}
```

**Benefits:**
- **Power delta threshold** - Reduces I2C traffic
- **Automatic rate limiting** - Built into framework
- **Enhanced telemetry** - Automatic logging
- **Error recovery** - Graceful degradation

### Servo Extensions

```kotlin
// ServoEx provides position control with units
servo.setPosition(45, AngleUnit.DEGREES)  // Instead of 0.0-1.0

// Continuous rotation servos
crServo.setPower(0.5)
```

---

## Task-Based Programming

### Task Types

#### 1. Control Tasks
Direct manual control from gamepad:

```kotlin
// Lift control with joystick
Proto.lift.tasks.control { -gamepad2.lsy.toDouble() }

// Conditional control
Proto.intake.tasks.control {
    if (gamepad2.x) 1.0 else if (gamepad2.y) -1.0 else 0.0
}
```

#### 2. Position Tasks
Move to specific positions:

```kotlin
// Home the lift
Proto.lift.tasks.home()

// Move to position
Proto.lift.tasks.moveTo(targetPosition)
```

#### 3. Delta Control Tasks
Time-based adjustments:

```kotlin
// Rotation with time-based delta
Proto.rotator.tasks.controlDelta {
    gamepad2.rsy.toDouble() * (timer.deltaTime() to Seconds)
}
```

#### 4. Autonomous Tasks
Pre-programmed sequences:

```kotlin
// Drive to position
drive.tasks.goTo(Pose2d(24, 0, 0))

// Follow path
drive.tasks.followPath(path)
```

### Task Composition

Tasks can be combined in powerful ways:

```kotlin
// Sequential execution
task1.then(task2).then(task3)

// Parallel execution
task1.parallel(task2)

// Conditional execution
task.finishIf { condition }

// Timeout
task.timeout(5.0 to Seconds)

// Repeat
task.repeat(3)
```

---

## Advanced Features

### 1. RoadRunner Integration

BunyipsFTC deeply integrates RoadRunner for autonomous:

```kotlin
// Define drive parameters
drive: MecanumDrive = MecanumDrive(
    DriveModel(...),
    MecanumGains(...),
    MotionProfile(...)
)

// Use in autonomous
drive.tasks.goTo(Pose2d(x, y, heading))
drive.tasks.followPath(path)
```

**Benefits:**
- Smooth, accurate autonomous movement
- Built-in localization
- Path following with splines
- Velocity control

### 2. Two-Wheel Odometry

They use **dead wheel encoders** for precise localization:

```kotlin
// Dead wheel setup
hw.pe = getHardware("fr", RawEncoder::class.java) {
    it.direction = DcMotorSimple.Direction.FORWARD
}
hw.ppe = getHardware("br", RawEncoder::class.java) {
    it.direction = DcMotorSimple.Direction.REVERSE
}

// Two-wheel localizer
val localizer = TwoWheelLocalizer(
    hw.pe, hw.ppe, hw.imu,
    inPerTick = ENCODER_RESOLUTION
)
```

### 3. Field-Centric Drive

Built-in support for field-centric driving:

```kotlin
// Field-centric drive task
HolonomicVectorDriveTask(gamepad1, Proto.drive)
    .withFieldCentric(true)
    .setAsDefaultTask()
```

### 4. Advanced Button Handling

Rich button control API:

```kotlin
// Rising edge detection
driver() whenRising Controls.A run task

// Falling edge
driver() whenFalling Controls.A run task

// Pressed (held)
driver() whenPressed Controls.A run task

// Analog trigger with condition
operator() whenRising (Controls.Analog.RIGHT_TRIGGER to { v -> v == 1.0f })
    run task
```

### 5. Feedforward Control

Elevator feedforward for gravity compensation:

```kotlin
val feedforward = ElevatorFeedforward(kG, kV, kA)

// Automatically applied in HoldableActuator
lift.setFeedforward(feedforward)
```

### 6. PID Control

Built-in PID controllers:

```kotlin
val pid = PController(kP)

// Use in actuator
actuator.setPID(pid)
```

---

## What We Can Learn

### 1. Hardware Configuration Pattern

**Current Approach:**
```java
@Override
public void runOpMode() {
    // Repeated in every OpMode
    motor = hardwareMap.get(DcMotor.class, "motor");
    servo = hardwareMap.get(Servo.class, "servo");
    // ...
}
```

**Better Approach (Inspired by BunyipsFTC):**
```java
// RobotHardware.java - Single configuration class
public class RobotHardware {
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx lift;
    public Servo claw;

    private HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftDrive = initMotor("left_drive", DcMotor.Direction.REVERSE);
        rightDrive = initMotor("right_drive", DcMotor.Direction.FORWARD);
        lift = initMotor("lift", DcMotor.Direction.FORWARD);
        claw = hwMap.get(Servo.class, "claw");
    }

    private DcMotorEx initMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }
}

// In OpMode
@TeleOp(name="My Teleop")
public class MyTeleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);  // One line!

        waitForStart();

        while (opModeIsActive()) {
            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);
        }
    }
}
```

**Benefits:**
- ✅ Configure once, use everywhere
- ✅ Consistent motor settings across all OpModes
- ✅ Easier to maintain
- ✅ Centralized error handling

### 2. Subsystem Pattern

**Current Approach:**
```java
// Everything in one OpMode
public class MyTeleop extends OpMode {
    private DcMotor liftMotor;
    private Servo claw;

    private enum LiftState { DOWN, MOVING_UP, UP, MOVING_DOWN }
    private LiftState liftState = LiftState.DOWN;

    @Override
    public void loop() {
        // Lift state machine mixed with drive code
        switch (liftState) {
            case DOWN:
                // ...
                break;
            // ... lots of code
        }

        // Drive code mixed with lift code
        drive();
    }
}
```

**Better Approach:**
```java
// LiftSubsystem.java
public class LiftSubsystem {
    private DcMotorEx motor;
    private ElapsedTime timer = new ElapsedTime();

    private enum State { DOWN, MOVING_UP, UP, MOVING_DOWN }
    private State state = State.DOWN;

    public LiftSubsystem(DcMotorEx motor) {
        this.motor = motor;
    }

    public void manualControl(double power) {
        motor.setPower(power);
    }

    public void raiseToHigh() {
        state = State.MOVING_UP;
        timer.reset();
    }

    public void periodic() {
        // State machine logic isolated
        switch (state) {
            case DOWN:
                motor.setPower(0);
                break;
            case MOVING_UP:
                motor.setPower(0.8);
                if (timer.seconds() > 2.0) {
                    state = State.UP;
                }
                break;
            // ...
        }
    }
}

// In OpMode
@TeleOp(name="Subsystem Example")
public class SubsystemTeleop extends OpMode {
    private DriveSubsystem drive;
    private LiftSubsystem lift;
    private IntakeSubsystem intake;

    @Override
    public void init() {
        drive = new DriveSubsystem(/* motors */);
        lift = new LiftSubsystem(/* motor */);
        intake = new IntakeSubsystem(/* servo */);
    }

    @Override
    public void loop() {
        // Clean separation!
        drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        lift.manualControl(-gamepad2.left_stick_y);
        intake.setPower(gamepad2.right_trigger);

        // Update all subsystems
        drive.periodic();
        lift.periodic();
        intake.periodic();
    }
}
```

### 3. Power Delta Threshold

Reduce I2C traffic by only sending power updates when they change significantly:

```java
public class SmartMotor {
    private DcMotorEx motor;
    private double lastPower = 0;
    private double deltaThreshold = 0.02;  // 2% change required

    public SmartMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if (Math.abs(power - lastPower) > deltaThreshold) {
            motor.setPower(power);
            lastPower = power;
        }
    }
}
```

### 4. Builder Pattern for Configuration

Make configuration more readable:

```java
public class MotorBuilder {
    private DcMotorEx motor;

    public MotorBuilder(HardwareMap hwMap, String name) {
        motor = hwMap.get(DcMotorEx.class, name);
    }

    public MotorBuilder setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
        return this;
    }

    public MotorBuilder setBrakeMode() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    public MotorBuilder withEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return this;
    }

    public DcMotorEx build() {
        return motor;
    }
}

// Usage
leftDrive = new MotorBuilder(hardwareMap, "left_drive")
    .setDirection(DcMotor.Direction.REVERSE)
    .setBrakeMode()
    .withEncoder()
    .build();
```

### 5. Declarative Control Mapping

Instead of if/else chains:

```java
// Current approach - lots of if statements
if (gamepad1.a) {
    action1();
} else if (gamepad1.b) {
    action2();
} else if (gamepad1.x) {
    action3();
}

// Better - map-based approach
Map<BooleanSupplier, Runnable> buttonMap = new HashMap<>();
buttonMap.put(() -> gamepad1.a, () -> action1());
buttonMap.put(() -> gamepad1.b, () -> action2());
buttonMap.put(() -> gamepad1.x, () -> action3());

// In loop
for (Map.Entry<BooleanSupplier, Runnable> entry : buttonMap.entrySet()) {
    if (entry.getKey().getAsBoolean()) {
        entry.getValue().run();
    }
}
```

---

## Implementation Examples

### Example 1: Shared Robot Hardware

```java
/**
 * RobotConfig.java - Single source of truth for robot hardware
 */
public class RobotConfig {
    // Hardware instances
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx lift;
    public Servo claw;
    public IMU imu;

    private HardwareMap hwMap;
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        try {
            initDrive();
            initLift();
            initClaw();
            initIMU();

            telemetry.addData("Status", "Hardware Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addData("Status", "Hardware Init Failed!");
        }

        telemetry.update();
    }

    private void initDrive() {
        leftFront = configureMotor("left_front", DcMotor.Direction.REVERSE);
        rightFront = configureMotor("right_front", DcMotor.Direction.FORWARD);
        leftBack = configureMotor("left_back", DcMotor.Direction.REVERSE);
        rightBack = configureMotor("right_back", DcMotor.Direction.FORWARD);
    }

    private void initLift() {
        lift = configureMotor("lift", DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initClaw() {
        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(0.5);  // Neutral position
    }

    private void initIMU() {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
    }

    private DcMotorEx configureMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }
}

// Usage in TeleOp
@TeleOp(name="My Teleop")
public class MyTeleop extends LinearOpMode {
    RobotConfig robot = new RobotConfig();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            // Use robot.leftFront, robot.lift, etc.
        }
    }
}

// Usage in Autonomous
@Autonomous(name="My Auto")
public class MyAuto extends LinearOpMode {
    RobotConfig robot = new RobotConfig();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        // Use same hardware configuration!
    }
}
```

### Example 2: Lift Subsystem

```java
/**
 * LiftSubsystem.java - Encapsulates lift hardware and logic
 */
public class LiftSubsystem {
    private DcMotorEx motor;
    private ElapsedTime timer = new ElapsedTime();

    // Constants
    private static final int LOW_POSITION = 0;
    private static final int MID_POSITION = 500;
    private static final int HIGH_POSITION = 1000;
    private static final double HOLD_POWER = 0.1;

    public enum State {
        MANUAL,      // Manual control
        MOVING,      // Moving to position
        HOLDING      // Holding position
    }

    private State currentState = State.MANUAL;
    private int targetPosition = 0;

    public LiftSubsystem(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Manual control
    public void setPower(double power) {
        currentState = State.MANUAL;
        motor.setPower(power);
    }

    // Position control
    public void goToLow() {
        setTargetPosition(LOW_POSITION);
    }

    public void goToMid() {
        setTargetPosition(MID_POSITION);
    }

    public void goToHigh() {
        setTargetPosition(HIGH_POSITION);
    }

    private void setTargetPosition(int position) {
        targetPosition = position;
        currentState = State.MOVING;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.8);
        timer.reset();
    }

    // Call this every loop
    public void periodic() {
        switch (currentState) {
            case MOVING:
                // Check if reached target
                if (!motor.isBusy() || timer.seconds() > 3.0) {
                    currentState = State.HOLDING;
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case HOLDING:
                // Hold position with minimal power
                motor.setPower(HOLD_POWER);
                break;
            case MANUAL:
                // Do nothing - manual control active
                break;
        }
    }

    public State getState() {
        return currentState;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(motor.getCurrentPosition() - targetPosition) < 10;
    }
}
```

---

## Comparison with Standard FTC

| Feature | Standard FTC | BunyipsFTC |
|---------|-------------|------------|
| **OpMode Style** | LinearOpMode / OpMode | CommandBasedOpMode |
| **Hardware Config** | Repeated in each OpMode | Single RobotConfig object |
| **Control Style** | Imperative (if/else) | Declarative (tasks/commands) |
| **Subsystems** | Manual implementation | Built-in subsystem classes |
| **Button Handling** | Manual state tracking | Automatic edge detection |
| **Autonomous** | Manual or basic | RoadRunner integrated |
| **Localization** | Manual or external | Built-in odometry |
| **Field-Centric** | Manual implementation | One-line enable |
| **Task Composition** | Manual | Built-in chaining |
| **Language** | Java only | Kotlin + Java |
| **Learning Curve** | 🟢 Easy | 🟡 Moderate |
| **Code Complexity** | Higher (more boilerplate) | Lower (less boilerplate) |
| **Power** | 🟡 Moderate | 🟢 Very High |

---

## Recommendations

### For Our Team

#### 1. Adopt Immediately ✅
- **Hardware Configuration Class** - Create RobotConfig.java
- **Subsystem Pattern** - Separate drive, lift, intake into classes
- **Power Delta Threshold** - Reduce I2C traffic
- **Builder Pattern** - Cleaner motor configuration

#### 2. Consider for Next Season 🤔
- **Command-Based Architecture** - Requires significant refactoring
- **Task System** - Complex but powerful
- **Kotlin Migration** - Modern language features
- **Full BunyipsLib** - External dependency management

#### 3. Not Recommended ❌
- **Complete Rewrite** - Too risky mid-season
- **Kotlin-Only** - Java works fine, no need to force
- **Over-Abstraction** - Keep it simple for beginners

### Implementation Plan

#### Phase 1: Hardware Abstraction (Now)
```java
1. Create RobotConfig.java with all hardware
2. Update existing OpModes to use RobotConfig
3. Test thoroughly
```

#### Phase 2: Subsystem Pattern (Next)
```java
1. Create DriveSubsystem.java
2. Create LiftSubsystem.java
3. Create IntakeSubsystem.java
4. Refactor OpModes to use subsystems
```

#### Phase 3: Advanced Features (Future)
```java
1. Add button edge detection utilities
2. Implement basic task system
3. Consider RoadRunner integration
4. Evaluate command-based architecture
```

---

## Learning Path for Beginner Teams

This section provides a structured progression for teams new to FTC programming who want to eventually work towards command-based architecture like BunyipsFTC.

### Phase 1: Java/Kotlin Fundamentals (Weeks 1-4)

Start with core programming concepts before diving into advanced architectures. **Current focus for our team.**

#### Essential Concepts
1. **Java Basics**
   - Classes and objects
   - Methods and parameters
   - Variables and data types
   - Control flow (if/else, loops)

2. **FTC OpMode Basics**
   - LinearOpMode structure
   - Hardware mapping
   - Telemetry for debugging
   - Gamepad input

3. **Optional: Kotlin Basics**
   - Kotlin vs Java syntax
   - Lambda expressions
   - Extension functions
   - Null safety

#### Recommended Resources
- **Java Tutorial for Beginners**: https://www.codecademy.com/learn/learn-java
- **FTC Official Docs**: https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-(Android-Studio).html
- **Game Manual 0**: https://gm0.org/en/latest/ (comprehensive FTC guide)
- **Kotlin for Java Developers**: https://kotlinlang.org/docs/java-to-kotlin-idioms-strings.html (if interested)

#### Practice Projects
```java
// Week 1-2: Basic Java & FTC
1. Create a simple TeleOp with tank drive
2. Add telemetry for motor positions
3. Map gamepad buttons to servo control
4. Display sensor readings

// Week 3-4: Object-Oriented Basics
5. Extract motor control into a separate class
6. Create a simple Robot class with init() method
7. Practice passing objects between methods
8. Understand inheritance (extend LinearOpMode)
```

### Phase 2: Software Design Patterns (Weeks 5-8)

Learn the foundational patterns that make BunyipsFTC powerful.

#### Essential Concepts
1. **Object-Oriented Design**
   - Encapsulation
   - Inheritance
   - Polymorphism
   - Abstraction

2. **Common Patterns**
   - Singleton pattern (RobotConfig)
   - Builder pattern (motor configuration)
   - Factory pattern (creating objects)
   - State pattern (FSM for autonomous)

3. **Code Organization**
   - Separation of concerns
   - DRY principle (Don't Repeat Yourself)
   - SOLID principles (basic understanding)

#### Recommended Resources
- **Head First Design Patterns**: Classic book, highly recommended
- **Refactoring Guru**: https://refactoring.guru/design-patterns (visual explanations)
- **Game Manual 0 - Software Design**: https://gm0.org/en/latest/docs/software/tutorials/index.html
- **FTC Design Patterns Guide**: Search "FTC software patterns" on Chief Delphi

#### Practice Projects
```java
// Week 5-6: Encapsulation & Classes
1. Create a RobotHardware class to store all motors/servos
2. Build a DriveBase class with drive methods
3. Implement a LiftController class
4. Test with existing TeleOp code

// Week 7-8: Builder & Factory Patterns
5. Implement builder pattern for motor config
6. Create factory methods for common hardware
7. Refactor RobotHardware to use builders
8. Compare before/after code complexity
```

### Phase 3: Subsystem Architecture (Weeks 9-12)

Understand how to break a robot into modular, maintainable components.

#### Essential Concepts
1. **Subsystem Pattern**
   - What is a subsystem?
   - Subsystem responsibilities
   - Periodic vs on-demand updates
   - Inter-subsystem communication

2. **Hardware Abstraction**
   - Separating hardware from logic
   - Config files vs hardcoded values
   - Mock hardware for testing
   - Hardware interface pattern

3. **State Management**
   - Subsystem state tracking
   - State machines within subsystems
   - Defensive programming

#### Recommended Resources
- **WPILib Subsystem Docs**: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html (FRC, but concepts apply)
- **BunyipsLib GitHub Examples**: https://github.com/Murray-Bridge-Bunyips/BunyipsLib/tree/master/src/main/java/org/murraybridgebunyips/bunyipslib
- **Game Manual 0 - Subsystems**: https://gm0.org/en/latest/docs/software/concepts/subsystems.html
- **Local BunyipsFTC Examples**: /Users/jameswang/projects/BunyipsFTC

#### Practice Projects
```java
// Week 9-10: First Subsystem
1. Create DriveSubsystem class
   - Init motors in constructor
   - Add drive(forward, turn) method
   - Add stop() method
   - Add telemetry() method
2. Update TeleOp to use DriveSubsystem

// Week 11-12: Multiple Subsystems
3. Create LiftSubsystem class
4. Create IntakeSubsystem class
5. Build RobotContainer to hold all subsystems
6. Practice subsystem coordination
```

### Phase 4: Command-Based Programming (Weeks 13-16)

Learn the paradigm that powers BunyipsFTC and FRC robots.

#### Essential Concepts
1. **Command Pattern**
   - What is a Command?
   - Command lifecycle (init, execute, end, isFinished)
   - Command scheduling
   - Command groups (sequential, parallel)

2. **Button Bindings**
   - Declarative vs imperative control
   - Edge detection (pressed, released, held)
   - Trigger conditions
   - Button -> Command mapping

3. **Default Commands**
   - What runs when nothing else does?
   - Subsystem requirements
   - Command interruption
   - Priority handling

#### Recommended Resources
- **WPILib Command-Based**: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
- **FTCLib Command System**: https://docs.ftclib.org/ftclib/command-base/command-system
- **BunyipsLib Task System**: Study their Task classes in GitHub
- **Video: Command-Based FTC**: Search YouTube for "FTC command based programming"

#### Practice Projects
```java
// Week 13-14: First Commands
1. Create DriveCommand class
   - Requires DriveSubsystem
   - Execute reads gamepad, calls subsystem
   - Never finishes (runs continuously)
2. Create LiftToPositionCommand
   - Requires LiftSubsystem
   - Execute moves lift
   - Finishes when at target

// Week 15-16: Button Bindings
3. Implement button edge detection utility
4. Map buttons to commands declaratively
5. Test command interruption
6. Create sequential command groups
```

### Phase 5: BunyipsFTC Integration (Weeks 17+)

**Prerequisites**: Completion of Phases 1-4, understanding of subsystems and commands

#### Essential Concepts
1. **BunyipsLib Architecture**
   - CommandBasedBunyipsOpMode
   - Task system vs Commands
   - RobotConfig pattern
   - Lambda-based control

2. **Advanced Features**
   - RoadRunner integration
   - Odometry and localization
   - Field-centric drive
   - Vision processing

3. **Kotlin Interop**
   - Using Kotlin and Java together
   - Converting Java classes to Kotlin
   - Kotlin DSL features
   - Extension functions

#### Recommended Resources
- **BunyipsLib Documentation**: https://github.com/Murray-Bridge-Bunyips/BunyipsLib/wiki (if available)
- **BunyipsLib Source Code**: Study the implementation directly
- **Local BunyipsFTC Code**: /Users/jameswang/projects/BunyipsFTC
- **Kotlin Documentation**: https://kotlinlang.org/docs/home.html

#### Learning Sequence
```
Week 17-18: BunyipsLib Setup
├─ Add BunyipsLib dependency
├─ Create RobotConfig object
├─ Convert one subsystem to BunyipsLib style
├─ Test with CommandBasedBunyipsOpMode
└─ Verify basic functionality

Week 19-20: Task System
├─ Convert commands to tasks
├─ Implement button bindings with whenPressed/whenReleased
├─ Add default tasks
├─ Test task interruption and scheduling

Week 21-22: Advanced Features
├─ Integrate RoadRunner for autonomous
├─ Add odometry for position tracking
├─ Implement field-centric drive
├─ Explore vision processing options

Week 23+: Competition Development
├─ Build complete autonomous routines
├─ Optimize TeleOp control
├─ Add advanced features (vision, localization)
└─ Iterate based on testing
```

### Learning Resources by Category

#### Books (Highly Recommended)
- **Head First Design Patterns** by Freeman & Freeman (OOP patterns)
- **Clean Code** by Robert C. Martin (code quality)
- **Effective Java** by Joshua Bloch (Java best practices)
- **Kotlin in Action** by Jemerov & Isakova (if learning Kotlin)

#### Online Courses
- **Codecademy Learn Java**: https://www.codecademy.com/learn/learn-java
- **JetBrains Academy**: https://www.jetbrains.com/academy/ (interactive learning)
- **Coursera Object-Oriented Design**: Search "Object Oriented Design coursera"

#### FTC-Specific Resources
- **Game Manual 0**: https://gm0.org (THE essential FTC resource)
- **FTC Docs**: https://ftc-docs.firstinspires.org
- **FTCLib Documentation**: https://docs.ftclib.org
- **WPILib Docs**: https://docs.wpilib.org (FRC, but patterns translate)

#### Video Tutorials
- **FTC Official YouTube**: https://www.youtube.com/@FIRSTTechChallenge
- **Wizards.exe**: https://www.youtube.com/@wizards.exe (excellent FTC tutorials)
- **FTC Java Programming**: Search YouTube for comprehensive series
- **Kotlin Tutorial Series**: JetBrains official YouTube channel

#### Community & Forums
- **FTC Discord**: https://discord.gg/first-tech-challenge
- **Chief Delphi FTC**: https://www.chiefdelphi.com/c/first-programs/first-tech-challenge/179
- **Reddit r/FTC**: https://reddit.com/r/FTC
- **Stack Overflow**: Tag questions with `first-tech-challenge`

### Recommended Learning Path Timeline

```
Complete Beginner → BunyipsFTC Command-Based Proficiency

Month 1-2: Java & FTC Fundamentals
├─ Master Java basics (classes, methods, objects)
├─ Understand OpMode structure
├─ Create simple TeleOp and Autonomous
└─ Study FTC SDK sample code

Month 3-4: Design Patterns & OOP
├─ Learn encapsulation and abstraction
├─ Study common design patterns
├─ Practice builder and factory patterns
└─ Refactor code to use patterns

Month 5-6: Subsystem Architecture
├─ Implement subsystem pattern
├─ Create DriveSubsystem, LiftSubsystem, etc.
├─ Build RobotContainer/RobotConfig
└─ Understand hardware abstraction

Month 7-8: Command-Based Basics
├─ Learn command pattern theory
├─ Implement basic commands
├─ Add button bindings
└─ Practice command scheduling

Month 9+: BunyipsFTC Integration
├─ Set up BunyipsLib
├─ Migrate to Task system
├─ Add advanced features (RoadRunner, vision)
└─ Build competition-ready code

Note: Our team is currently at Month 1-2 level.
      BunyipsFTC requires strong OOP foundation first.
```

### Study Tips for Beginner Teams

1. **Master Java First**: BunyipsFTC heavily uses advanced Java/Kotlin features. Strong fundamentals are essential.

2. **Read Professional Code**: Study BunyipsLib source code on GitHub. See how experienced teams structure projects.

3. **Small Refactors**: Don't rewrite everything at once. Gradually introduce patterns (start with RobotConfig).

4. **Pair Programming**: Work together when learning new concepts. One codes, one reviews and asks questions.

5. **Code Reviews**: Review each other's code to spread knowledge and catch issues early.

6. **Document Everything**: Comment your code extensively while learning. Future you will thank present you.

7. **Test Incrementally**: After each refactor, test thoroughly before moving on.

### Common Beginner Mistakes to Avoid

❌ **Jumping to command-based too early** → Master subsystems first
❌ **Not understanding OOP** → Study classes, objects, and inheritance thoroughly
❌ **Over-engineering** → Start simple, add complexity only when needed
❌ **Copying without understanding** → Type out code yourself, understand each line
❌ **Ignoring testing** → Test small changes frequently
❌ **Skipping design patterns** → Learn why patterns exist before using them
❌ **Mixing paradigms inconsistently** → Stick with one approach (imperative OR declarative)

### Key Milestones for Beginner Teams

✅ **Milestone 1**: Create a simple Java class with methods and fields
✅ **Milestone 2**: Extract hardware configuration into RobotConfig class
✅ **Milestone 3**: Implement your first subsystem (DriveSubsystem)
✅ **Milestone 4**: Create a working command that uses a subsystem
✅ **Milestone 5**: Set up button bindings with edge detection
✅ **Milestone 6**: Build a complete command-based TeleOp
✅ **Milestone 7**: (Advanced) Successfully use BunyipsLib in competition

### Gradual Adoption Strategy

BunyipsFTC's architecture is complex. Here's how to adopt it gradually:

#### Stage 1: Hardware Organization (Start Here)
```java
// Create RobotConfig.java - centralized hardware
public class RobotConfig {
    public static DcMotor leftFront;
    public static DcMotor rightFront;
    // ... all hardware

    public static void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        // ... configure all hardware
    }
}

// Use in OpModes
public void runOpMode() {
    RobotConfig.init(hardwareMap);
    // Use RobotConfig.leftFront instead of hardwareMap
}
```

#### Stage 2: Subsystem Extraction
```java
// Create DriveSubsystem.java
public class DriveSubsystem {
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Init motors
    }

    public void drive(double forward, double strafe, double turn) {
        // Calculate and set motor powers
    }
}

// Use in OpModes
DriveSubsystem drive = new DriveSubsystem(hardwareMap);
drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
```

#### Stage 3: Command Pattern (Future)
```java
// Eventually evolve to command-based
public class TeleOpCommand extends Command {
    private DriveSubsystem drive;

    @Override
    public void execute() {
        drive.drive(/* gamepad values */);
    }
}
```

#### Stage 4: BunyipsLib Integration (Advanced)
```kotlin
// Final form: BunyipsLib command-based
@TeleOp
class MainTeleOp : CommandBasedBunyipsOpMode() {
    override fun assignCommands() {
        HolonomicVectorDriveTask(gamepad1, Proto.drive).setAsDefaultTask()
    }
}
```

### When to Use Each Approach

```
Decision Tree:

Is your team new to Java/programming?
├─ YES → Start with standard LinearOpMode (Phase 1-2)
└─ NO → Continue

Do you understand OOP (classes, objects, inheritance)?
├─ NO → Focus on design patterns first (Phase 2-3)
└─ YES → Continue

Have you implemented subsystems?
├─ NO → Start with subsystem architecture (Phase 3)
└─ YES → Continue

Do you understand command pattern?
├─ NO → Learn command-based basics (Phase 4)
└─ YES → Consider BunyipsFTC (Phase 5)

Do you need advanced features (RoadRunner, vision)?
├─ NO → Stick with simpler frameworks
└─ YES → BunyipsFTC is a good fit
```

### Comparison: When to Choose BunyipsFTC

**Choose BunyipsFTC if:**
- ✅ Team has strong Java/Kotlin experience
- ✅ Building a complex robot with many subsystems
- ✅ Need advanced autonomous (RoadRunner integration)
- ✅ Want professional-level code organization
- ✅ Have time to learn command-based architecture

**Stick with Standard FTC if:**
- ❌ Team is learning programming basics
- ❌ Building a simple robot (2-3 motors)
- ❌ Need quick turnaround (limited time)
- ❌ Prefer imperative, straightforward control
- ❌ Don't need advanced features

**Middle Ground (Adopt Patterns Only):**
- 🟡 Use RobotConfig pattern without full framework
- 🟡 Implement subsystems in standard OpModes
- 🟡 Add builder pattern for cleaner hardware setup
- 🟡 Gradually introduce patterns as team skill grows

---

## Conclusion

BunyipsFTC demonstrates **professional-level software engineering** in FTC:

### Key Takeaways

1. **Hardware Abstraction** - Configure once, use everywhere
2. **Subsystem Pattern** - Encapsulate robot components
3. **Declarative Programming** - Less code, more clarity
4. **Task-Based Control** - Composable robot actions
5. **Advanced Autonomous** - RoadRunner + Odometry

### What We Should Do

✅ **Adopt their patterns** - Hardware config and subsystems
✅ **Learn from their structure** - Professional code organization
⚠️ **Don't blindly copy** - They have years of experience
⚠️ **Start simple** - Incremental improvements

### Resources

- **BunyipsLib GitHub:** https://github.com/Murray-Bridge-Bunyips/BunyipsLib
- **BunyipsFTC Repository:** https://github.com/Murray-Bridge-Bunyips/BunyipsFTC
- **Local BunyipsFTC Path:** /Users/jameswang/projects/BunyipsFTC
- **Their Documentation:** (Check their repo README)

---

**Remember:** BunyipsFTC is **years ahead** in software engineering. We should:
- Learn from their patterns
- Adopt what makes sense for us
- Keep our code maintainable
- Focus on competition success

**Bottom Line:** Steal their best ideas, but keep it simple! 🚀

---

**Document Version:** 1.0
**Last Updated:** 2025-11-09
**Next Review:** Start of next season
