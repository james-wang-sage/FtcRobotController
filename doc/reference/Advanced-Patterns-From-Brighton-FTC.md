# Advanced Patterns from Brighton-FTC Implementation

**Analysis Date:** November 9, 2025
**Source Repository:** /Users/jameswang/projects/2026
**Key Focus:** Pedro Pathing autonomous framework integration

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Pedro Pathing Framework Overview](#pedro-pathing-framework-overview)
3. [Architecture Analysis](#architecture-analysis)
4. [Key Patterns and Implementations](#key-patterns-and-implementations)
5. [Comparison with Standard FTC and BunyipsFTC](#comparison-with-standard-ftc-and-bunyipsftc)
6. [Implementation Recommendations](#implementation-recommendations)
7. [Code Examples](#code-examples)
8. [References](#references)

---

## Executive Summary

Brighton-FTC's implementation centers around **Pedro Pathing**, a sophisticated autonomous path following library developed by FTC team 10158. Unlike BunyipsFTC's command-based architecture, Brighton-FTC focuses specifically on **advanced autonomous navigation** using Bézier curves, multi-vector control, and comprehensive tuning tools.

### Key Takeaways

- **Specialized Autonomous Framework**: Pedro Pathing provides superior path following compared to basic FTC approaches
- **Mathematical Precision**: Uses Bézier curves for smooth, efficient trajectories
- **Comprehensive Tuning Suite**: 15+ tuning OpModes for calibration and optimization
- **Holonomic Drive Required**: Designed specifically for mecanum/omni drives (not tank drives)
- **FTCLib Integration**: Uses FTCLib for enhanced gamepad handling
- **Dashboard Integration**: Built-in visual debugging with Panels Field

---

## Pedro Pathing Framework Overview

### What is Pedro Pathing?

Pedro Pathing is an advanced path follower that revolutionizes autonomous navigation through:

1. **Bézier Curve Path Generation**: Smooth, mathematically-defined trajectories
2. **Multi-Vector Control System**: Four vectors calculate optimal wheel powers
3. **Real-time Correction**: Continuous localization and path deviation correction
4. **Dynamic Path Creation**: Generate paths on-the-fly during autonomous

### Core Architecture

```
Pedro Pathing System
├── Follower (main controller)
│   ├── Path Following Engine
│   ├── Localization Integration
│   └── PIDF Controllers
├── Path Components
│   ├── BezierCurve (curved paths)
│   ├── BezierLine (straight paths)
│   └── PathChain (combined paths)
├── Tuning System
│   ├── Localization Tuners
│   ├── Velocity Tuners
│   └── PIDF Tuners
└── Visualization
    └── Panels Field Integration
```

### Hardware Requirements

Pedro Pathing has **strict hardware requirements**:

| Requirement | Details | Why Required |
|-------------|---------|--------------|
| **Omnidirectional Drive** | Mecanum, X-drive, Swerve | Enables full holonomic movement |
| **Localization** | Dead wheels, Pinpoint, OTOS, or encoders | Real-time position tracking |
| **Android Studio** | Full Java development | OnBot Java/Blocks incompatible |
| **Two-Wheel Odometry** | Minimum setup | Parallel + perpendicular encoders |

**Important**: Tank drives are **NOT compatible** with Pedro Pathing.

---

## Architecture Analysis

### 1. Constants Pattern

Brighton-FTC uses a **centralized constants class** for all Pedro Pathing configuration:

**File**: `Constants.java` (TeamCode/pedroPathing/)

```java
public class Constants {
    // Motion constraints
    public static FollowerConstants followerConstants = new FollowerConstants().mass(5);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Drive configuration
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    // Localization configuration
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightRear")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
            );

    // Factory method pattern
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
```

**Key Pattern**: **Builder Pattern + Factory Method**
- Fluent configuration API
- Single factory method creates fully configured Follower
- All robot-specific parameters in one place

### 2. Comprehensive Tuning System

Brighton-FTC includes **15+ specialized tuning OpModes** organized in a hierarchical menu:

**File**: `Tuning.java` (1325 lines of tuning infrastructure)

#### Tuning Categories

```
Tuning Menu
├── Localization
│   ├── Localization Test (real-time position display)
│   ├── Forward Tuner (calibrate forward encoder)
│   ├── Lateral Tuner (calibrate strafe encoder)
│   └── Turn Tuner (calibrate rotation)
├── Automatic
│   ├── Forward Velocity Tuner (measure max forward speed)
│   ├── Lateral Velocity Tuner (measure max strafe speed)
│   ├── Forward Zero Power Acceleration (measure deceleration)
│   └── Lateral Zero Power Acceleration (measure lateral deceleration)
├── Manual
│   ├── Translational Tuner (test lateral PIDF)
│   ├── Heading Tuner (test rotational PIDF)
│   ├── Drive Tuner (test forward PIDF)
│   └── Centripetal Tuner (test curve correction)
└── Tests
    ├── Line (straight path test)
    ├── Triangle (multi-segment test)
    └── Circle (continuous curve test)
```

#### Tuning OpMode Pattern

All tuning OpModes follow a **consistent structure**:

```java
class LocalizationTest extends OpMode {
    @Override
    public void init() {}

    @Override
    public void init_loop() {
        telemetryM.debug("Instructions for this tuner...");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void loop() {
        follower.update();
        // Collect data
        telemetryM.debug("Measurement: " + value);
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}
```

**Key Features**:
- Clear user instructions in init_loop
- Real-time visual feedback with field drawing
- Live telemetry with measurement calculations
- Optional temporary value setting with gamepad

### 3. FTCLib Integration

Brighton-FTC uses **FTCLib** for enhanced gamepad handling:

**File**: `PSButtons.java`

```java
public class PSButtons {
    // PlayStation button mapping for better driver familiarity
    public static final GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
    public static final GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
    public static final GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
    public static final GamepadKeys.Button CROSS = GamepadKeys.Button.A;

    public static final GamepadKeys.Button SHARE = GamepadKeys.Button.BACK;
    public static final GamepadKeys.Button OPTIONS = GamepadKeys.Button.START;
}
```

**Why This Matters**:
- Drivers familiar with PlayStation controllers have intuitive button names
- Reduces driver confusion during competition
- FTCLib provides advanced gamepad features (edge detection, debouncing)

### 4. Dependency Management

Brighton-FTC integrates multiple external libraries:

**File**: `build.dependencies.gradle`

```gradle
repositories {
    maven { url = 'https://maven.pedropathing.com' }
}

dependencies {
    implementation 'com.pedropathing:ftc:2.0.0'
    implementation 'com.pedropathing:telemetry:0.0.6'
}
```

**File**: `TeamCode/build.gradle`

```gradle
dependencies {
    implementation project(':FtcRobotController')
    implementation 'org.openftc:apriltag:2.1.0'
    implementation 'org.ftclib.ftclib:core:2.1.1'
    implementation 'org.openftc:opencv-repackaged-bundled-dylibs:4.10.0-A'
}
```

**Libraries Used**:
- **Pedro Pathing 2.0**: Path following framework
- **FTCLib 2.1.1**: Enhanced FTC utilities
- **OpenFTC AprilTag**: Vision processing
- **OpenCV**: Computer vision support

---

## Key Patterns and Implementations

### Pattern 1: Selectable OpMode Menu

**Implementation**: Hierarchical tuning menu with folders

```java
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Forward Tuner", ForwardTuner::new);
                // ... more entries
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                // ... more entries
            });
        });
    }
}
```

**Benefits**:
- Single OpMode provides access to all tuning tools
- Organized by category for easy navigation
- Reduces Driver Station OpMode clutter

### Pattern 2: Pose-Based Navigation

**Pedro Pathing Coordinate System**:

```
Field Coordinate System (144" x 144")
┌─────────────────────────────────┐ (144, 144)
│                                 │
│         Blue Alliance           │
│                                 │
│  (0,0)                          │
└─────────────────────────────────┘ (144, 0)
  Red Alliance
```

**Pose Definition**:

```java
// Pose(x, y, heading)
Pose startPose = new Pose(0, 0, Math.toRadians(0));
Pose scorePose = new Pose(24, 48, Math.toRadians(90));
```

### Pattern 3: Path Building with Bézier Curves

**BezierLine** (Straight Path):

```java
Path straightPath = new Path(
    new BezierLine(
        new Pose(0, 0),        // Start
        new Pose(48, 0)        // End
    )
);
straightPath.setConstantHeadingInterpolation(0);
```

**BezierCurve** (Curved Path):

```java
Path curvedPath = new Path(
    new BezierCurve(
        new Pose(0, 0),           // Start
        new Pose(24, 0),          // Control point 1
        new Pose(24, 24),         // Control point 2
        new Pose(48, 24)          // End
    )
);
```

**PathChain** (Multiple Segments):

```java
PathChain autoPath = follower.pathBuilder()
    .addPath(new BezierLine(startPose, intermediatePose))
    .setLinearHeadingInterpolation(startPose.getHeading(), intermediatePose.getHeading())
    .addPath(new BezierCurve(intermediatePose, controlPoint, endPose))
    .setTangentHeadingInterpolation()
    .build();

follower.followPath(autoPath);
```

### Pattern 4: Finite State Machine for Autonomous

**Typical Autonomous Structure**:

```java
@Autonomous(name = "Red Alliance Auto")
public class RedAuto extends OpMode {
    private Follower follower;
    private PathChain scorePath, parkPath;

    private enum AutoState {
        DRIVE_TO_BASKET,
        SCORE_SAMPLE,
        DRIVE_TO_PARK,
        PARK,
        IDLE
    }

    private AutoState currentState = AutoState.DRIVE_TO_BASKET;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build all paths during init
        scorePath = follower.pathBuilder()
            .addPath(new BezierLine(startPose, basketPose))
            .setConstantHeadingInterpolation(0)
            .build();

        parkPath = follower.pathBuilder()
            .addPath(new BezierCurve(basketPose, controlPoint, parkPose))
            .setTangentHeadingInterpolation()
            .build();
    }

    @Override
    public void start() {
        follower.followPath(scorePath);
    }

    @Override
    public void loop() {
        follower.update();

        switch (currentState) {
            case DRIVE_TO_BASKET:
                if (!follower.isBusy()) {
                    currentState = AutoState.SCORE_SAMPLE;
                    // Start scoring actions
                }
                break;

            case SCORE_SAMPLE:
                if (scoringComplete()) {
                    currentState = AutoState.DRIVE_TO_PARK;
                    follower.followPath(parkPath);
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
                // Final adjustments
                currentState = AutoState.IDLE;
                break;

            case IDLE:
                // Do nothing
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.update();
    }
}
```

**Key State Machine Features**:
- **Non-blocking**: `followPath()` returns immediately, loop continues
- **Condition-based transitions**: Check `follower.isBusy()` or custom conditions
- **Flexible**: Can add time-based, position-based, or sensor-based transitions

### Pattern 5: Visual Debugging with Panels Field

**Field Visualization**:

```java
public static void drawCurrentAndHistory() {
    Drawing.drawPoseHistory(poseHistory);
    Drawing.drawRobot(follower.getPose());
    Drawing.sendPacket();
}

public static void drawRobot(Pose pose) {
    // Draw robot circle
    panelsField.setStyle(robotLook);
    panelsField.moveCursor(pose.getX(), pose.getY());
    panelsField.circle(ROBOT_RADIUS);

    // Draw heading line
    Vector heading = pose.getHeadingAsUnitVector();
    heading.setMagnitude(heading.getMagnitude() * ROBOT_RADIUS);
    panelsField.line(
        pose.getX() + heading.getXComponent(),
        pose.getY() + heading.getYComponent()
    );
}
```

**Benefits**:
- Real-time robot position visualization
- Path history trail
- Heading indicator
- Path preview

### Pattern 6: Empirical Velocity Measurement

**Forward Velocity Tuner** (measures actual robot capabilities):

```java
class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;  // inches
    public static double RECORD_NUMBER = 10;  // samples to average

    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();

        if (!end) {
            if (Math.abs(follower.getPose().getX()) > DISTANCE) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(1, 0, 0, true);  // Max forward
                double currentVelocity = Math.abs(
                    follower.poseTracker.getLocalizer().getVelocity().getX()
                );
                velocities.add(currentVelocity);
                velocities.remove(0);  // Keep only recent velocities
            }
        } else {
            // Calculate average
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();

            telemetryM.debug("Forward Velocity: " + average);
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.setXVelocity(average);  // Apply temporarily
            }
        }
    }
}
```

**Why This Matters**:
- Measures **actual** robot performance (not theoretical)
- Accounts for friction, battery voltage, motor wear
- Provides accurate velocity constraints for path following
- Averaging reduces noise and outliers

### Pattern 7: Zero Power Acceleration Measurement

**Measures natural deceleration** when power is cut:

```java
class ForwardZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;  // Target velocity

    @Override
    public void loop() {
        follower.update();

        if (!end) {
            if (!stopping) {
                // Accelerate to target velocity
                if (follower.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = follower.getVelocity().dot(heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0, 0, 0, true);  // CUT POWER
                }
            } else {
                // Measure deceleration
                double currentVelocity = follower.getVelocity().dot(heading);
                double deceleration = (currentVelocity - previousVelocity) /
                    ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9));
                accelerations.add(deceleration);

                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                if (currentVelocity < THRESHOLD) {
                    end = true;
                }
            }
        } else {
            // Average and display
            double average = 0;
            for (double accel : accelerations) {
                average += accel;
            }
            average /= accelerations.size();

            telemetryM.debug("Forward Zero Power Acceleration: " + average);
        }
    }
}
```

**Why This Matters**:
- Enables accurate **braking predictions**
- Path follower can pre-emptively reduce power for smooth stops
- Improves end-of-path precision
- Accounts for wheel friction and momentum

---

## Comparison with Standard FTC and BunyipsFTC

### Feature Comparison Matrix

| Feature | Standard FTC | BunyipsFTC | Brighton-FTC (Pedro Pathing) |
|---------|--------------|------------|------------------------------|
| **Primary Focus** | Basic control | Command architecture | Autonomous navigation |
| **Path Following** | Basic time/encoder | RoadRunner integration | Pedro Pathing (Bézier) |
| **Curve Generation** | Manual waypoints | Spline-based | Bézier curves |
| **Real-time Correction** | Manual PID | Built-in PID | Multi-vector PIDF |
| **Tuning Tools** | Manual testing | Some tuning | 15+ specialized tuners |
| **Visual Debugging** | Telemetry only | RoadRunner dashboard | Panels Field integration |
| **Hardware Abstraction** | None | RobotConfig pattern | Constants + Factory |
| **State Management** | Manual if/else | Command scheduling | FSM pattern |
| **Drive Type Support** | All | All (with adapters) | **Holonomic only** |
| **Learning Curve** | Low | Medium-High | Medium |
| **Localization** | Manual | Multiple options | Required (2-wheel min) |
| **Competition Focus** | TeleOp | Both | **Autonomous** |

### Philosophical Differences

#### Standard FTC
- **Philosophy**: Direct hardware control, learn fundamentals
- **Best For**: Beginners, simple robots, learning basics
- **Limitations**: Limited autonomous capabilities, manual tuning

#### BunyipsFTC
- **Philosophy**: Professional software architecture, scalability
- **Best For**: Complex robots, experienced programmers, full-season development
- **Limitations**: Steep learning curve, potentially over-engineered for simple tasks

#### Brighton-FTC (Pedro Pathing)
- **Philosophy**: Specialized autonomous excellence, mathematical precision
- **Best For**: Competitive autonomous, mecanum/omni drives, precise navigation
- **Limitations**: Requires holonomic drive, Android Studio only, autonomous-focused

### When to Use Each Approach

```
Decision Tree:

Do you have a holonomic drive (mecanum/omni)?
├─ NO → Use Standard FTC or BunyipsFTC
└─ YES → Continue
    │
    Is autonomous performance critical?
    ├─ NO → Use Standard FTC (simple) or BunyipsFTC (complex robot)
    └─ YES → Continue
        │
        Can you use Android Studio?
        ├─ NO → Use BunyipsFTC or Standard FTC
        └─ YES → Continue
            │
            Do you have localization hardware (dead wheels/OTOS)?
            ├─ NO → Use Standard FTC or BunyipsFTC
            └─ YES → Use Pedro Pathing (Brighton-FTC approach)
```

---

## Implementation Recommendations

### For Our Team (Current 2-Motor Tank Drive)

**Current Situation Analysis**:
- ✅ Have: Standard FTC SDK, 2-motor tank drive
- ❌ Missing: Holonomic drive, localization hardware
- 📊 Level: Learning FTC basics

**Recommendation**: **NOT COMPATIBLE - DO NOT IMPLEMENT PEDRO PATHING**

**Why**:
1. Pedro Pathing **requires holonomic drive** (mecanum/omni wheels)
2. Our 2-motor tank drive is **explicitly incompatible**
3. Would require complete robot rebuild

**Alternative Path**:
```
Current Season:
├─ Master tank drive with enhanced BasicOpMode_Linear.java ✅
├─ Learn encoder-based autonomous
├─ Implement basic PID control
└─ Gain competition experience

Future Season (if upgrading to mecanum):
├─ Hardware: Add 2 more motors + mecanum wheels
├─ Hardware: Add dead wheel odometry
├─ Software: Implement Pedro Pathing
└─ Benefit: Superior autonomous performance
```

### For Teams with Holonomic Drives

**Phase 1: Initial Setup** (Week 1)
```
1. Install Pedro Pathing dependency
   - Add maven repository
   - Add pedropathing:ftc:2.0.0

2. Create Constants.java
   - Configure motor names
   - Set motor directions
   - Configure localizer hardware

3. Test LocalizationTest OpMode
   - Verify position tracking
   - Check heading accuracy
```

**Phase 2: Tuning** (Weeks 2-3)
```
Localization Tuning:
1. Run ForwardTuner → Get forward multiplier
2. Run LateralTuner → Get strafe multiplier
3. Run TurnTuner → Get turn multiplier
4. Update Constants with values

Velocity Tuning:
5. Run ForwardVelocityTuner → Max forward speed
6. Run LateralVelocityTuner → Max strafe speed
7. Update PathConstraints

Deceleration Tuning:
8. Run ForwardZeroPowerAccelerationTuner
9. Run LateralZeroPowerAccelerationTuner
10. Update FollowerConstants

PIDF Tuning:
11. Run TranslationalTuner → Adjust lateral correction
12. Run HeadingTuner → Adjust rotation correction
13. Run DriveTuner → Adjust forward correction
14. Run CentripetalTuner → Adjust curve correction
```

**Phase 3: Path Testing** (Week 4)
```
1. Run Line test → Verify straight paths
2. Run Triangle test → Verify multi-segment paths
3. Run Circle test → Verify continuous curves
4. Iterate on PIDF values as needed
```

**Phase 4: Autonomous Development** (Weeks 5+)
```
1. Map field positions as Poses
2. Build paths in init()
3. Implement FSM for state management
4. Test incrementally (one state at a time)
5. Add scoring mechanisms
6. Practice autonomous routines
```

### Integration with Existing Code

**Option 1**: Pedro Pathing for Autonomous Only
```java
// Autonomous OpMode
@Autonomous(name = "Pedro Auto")
public class PedroAuto extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Build paths
    }

    @Override
    public void loop() {
        follower.update();
        // FSM logic
    }
}

// TeleOp - Standard FTC
@TeleOp(name = "Standard TeleOp")
public class StandardTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Use your existing tank/mecanum drive code
    }
}
```

**Option 2**: Pedro Pathing for Both (if holonomic)
```java
// TeleOp with Pedro
@TeleOp(name = "Pedro TeleOp")
public class PedroTeleOp extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,   // Forward/backward
            -gamepad1.left_stick_x,   // Strafe left/right
            -gamepad1.right_stick_x,  // Rotate
            true                      // Field-relative
        );
        follower.update();
    }
}
```

### Recommended Libraries for Integration

**If upgrading to mecanum drive**, consider this library stack:

```gradle
dependencies {
    // Core FTC SDK
    implementation project(':FtcRobotController')

    // Pedro Pathing for autonomous
    implementation 'com.pedropathing:ftc:2.0.0'
    implementation 'com.pedropathing:telemetry:0.0.6'

    // FTCLib for utilities
    implementation 'org.ftclib.ftclib:core:2.1.1'

    // Vision (if using AprilTag/color detection)
    implementation 'org.openftc:apriltag:2.1.0'
    implementation 'org.openftc:opencv-repackaged-bundled-dylibs:4.10.0-A'
}
```

---

## Code Examples

### Example 1: Simple Straight Path

```java
@Autonomous(name = "Simple Forward")
public class SimpleForward extends OpMode {
    private Follower follower;
    private Path forwardPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Simple 48-inch forward path
        forwardPath = new Path(
            new BezierLine(
                new Pose(0, 0),
                new Pose(48, 0)
            )
        );
        forwardPath.setConstantHeadingInterpolation(0);
    }

    @Override
    public void start() {
        follower.followPath(forwardPath);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();

        if (!follower.isBusy()) {
            requestOpModeStop();
        }
    }
}
```

### Example 2: Multi-Segment Autonomous with FSM

```java
@Autonomous(name = "Red Score and Park")
public class RedScoreAndPark extends OpMode {
    private Follower follower;
    private PathChain scorePath, parkPath;
    private Timer actionTimer = new Timer();

    // Define field positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose basketPose = new Pose(24, 48, Math.toRadians(90));
    private final Pose parkPose = new Pose(48, 72, Math.toRadians(180));

    private enum AutoState {
        DRIVE_TO_BASKET,
        RAISE_ARM,
        SCORE_SAMPLE,
        LOWER_ARM,
        DRIVE_TO_PARK,
        PARK,
        IDLE
    }

    private AutoState currentState = AutoState.DRIVE_TO_BASKET;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build path to basket
        scorePath = follower.pathBuilder()
            .addPath(new BezierCurve(
                startPose,
                new Pose(12, 24),  // Control point
                basketPose
            ))
            .setLinearHeadingInterpolation(
                startPose.getHeading(),
                basketPose.getHeading()
            )
            .build();

        // Build path to park
        parkPath = follower.pathBuilder()
            .addPath(new BezierLine(basketPose, parkPose))
            .setTangentHeadingInterpolation()
            .build();
    }

    @Override
    public void start() {
        follower.followPath(scorePath);
        actionTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        switch (currentState) {
            case DRIVE_TO_BASKET:
                if (!follower.isBusy()) {
                    currentState = AutoState.RAISE_ARM;
                    // Start arm raising
                    actionTimer.reset();
                }
                break;

            case RAISE_ARM:
                // Raise arm to scoring position
                if (actionTimer.seconds() > 1.5) {
                    currentState = AutoState.SCORE_SAMPLE;
                    actionTimer.reset();
                }
                break;

            case SCORE_SAMPLE:
                // Release sample
                if (actionTimer.seconds() > 0.5) {
                    currentState = AutoState.LOWER_ARM;
                    actionTimer.reset();
                }
                break;

            case LOWER_ARM:
                // Lower arm back down
                if (actionTimer.seconds() > 1.5) {
                    currentState = AutoState.DRIVE_TO_PARK;
                    follower.followPath(parkPath);
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
                // Final adjustments if needed
                currentState = AutoState.IDLE;
                break;

            case IDLE:
                // Do nothing
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }
}
```

### Example 3: Dynamic Path Generation

```java
@Autonomous(name = "Dynamic Scoring")
public class DynamicScoring extends OpMode {
    private Follower follower;
    private Pose targetPose;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        // Determine target based on alliance or sensor data
        targetPose = determineTargetPose();

        // Generate path on-the-fly
        PathChain dynamicPath = follower.pathBuilder()
            .addPath(new BezierLine(
                follower.getPose(),
                targetPose
            ))
            .setLinearHeadingInterpolation(
                follower.getPose().getHeading(),
                targetPose.getHeading()
            )
            .build();

        follower.followPath(dynamicPath);
    }

    @Override
    public void loop() {
        follower.update();

        // Can generate new paths mid-autonomous
        if (sensorDetectsObstacle()) {
            Pose avoidancePose = calculateAvoidancePoint();

            PathChain newPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                    follower.getPose(),
                    calculateControlPoint(),
                    avoidancePose
                ))
                .build();

            follower.followPath(newPath, true);  // Interrupt current path
        }
    }

    private Pose determineTargetPose() {
        // Logic to determine target
        return new Pose(48, 48, Math.toRadians(90));
    }
}
```

### Example 4: Integration with Subsystems

```java
@Autonomous(name = "Full Robot Auto")
public class FullRobotAuto extends OpMode {
    private Follower follower;

    // Subsystems (your existing hardware)
    private DcMotorEx armMotor;
    private Servo clawServo;

    // Paths
    private PathChain path1, path2;

    // State
    private enum AutoState {
        APPROACH_SAMPLE,
        GRAB_SAMPLE,
        SCORE_SAMPLE,
        PARK,
        IDLE
    }
    private AutoState state = AutoState.APPROACH_SAMPLE;
    private Timer timer = new Timer();

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize subsystems
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        clawServo = hardwareMap.get(Servo.class, "claw");

        // Build paths
        path1 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Pose(0, 0),
                new Pose(24, 0)
            ))
            .build();
    }

    @Override
    public void start() {
        follower.followPath(path1);
        timer.reset();
    }

    @Override
    public void loop() {
        // Update Pedro Pathing
        follower.update();

        // Update subsystems
        updateArm();

        // FSM
        switch (state) {
            case APPROACH_SAMPLE:
                if (!follower.isBusy()) {
                    state = AutoState.GRAB_SAMPLE;
                    clawServo.setPosition(0.0);  // Open claw
                    timer.reset();
                }
                break;

            case GRAB_SAMPLE:
                if (timer.seconds() > 0.5) {
                    clawServo.setPosition(1.0);  // Close claw
                    state = AutoState.SCORE_SAMPLE;
                    // Navigate to scoring position
                }
                break;

            // ... more states
        }

        telemetry.addData("State", state);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    private void updateArm() {
        // Your arm control logic
        if (state == AutoState.SCORE_SAMPLE) {
            armMotor.setPower(0.8);  // Raise arm
        } else {
            armMotor.setPower(0.0);
        }
    }
}
```

---

## References

### Official Documentation

1. **Pedro Pathing Official Docs**: https://pedropathing.com/docs/pathing
2. **FTCLib Documentation**: https://docs.ftclib.org/
3. **FTC Official Docs**: https://ftc-docs.firstinspires.org/

### Key Resources

- **Pedro Pathing GitHub**: https://github.com/Pedro-Pathing/PedroPathing
- **Pedro Pathing Tutorial Video**: https://www.youtube.com/watch?v=6v7QgzRhOwA
- **FTCLib GitHub**: https://github.com/FTCLib/FTCLib
- **OpenFTC AprilTag**: https://github.com/OpenFTC/EOCV-AprilTag-Plugin

### Learning Path

**For beginners considering Pedro Pathing**:
1. Master basic FTC programming first
2. Understand PID control concepts
3. Learn about Bézier curves (optional but helpful)
4. Watch Pedro Pathing tutorial videos
5. Complete tuning process methodically
6. Start with simple straight paths
7. Progress to multi-segment autonomous

### Community Resources

- **FTC Discord**: Ask questions in #programming channel
- **Chief Delphi FTC Forum**: https://www.chiefdelphi.com/forums/forumdisplay.php?f=179
- **Reddit r/FTC**: https://reddit.com/r/FTC
- **Local Brighton-FTC Path**: /Users/jameswang/projects/2026

---

## Learning Path for Beginner Teams

This section provides a structured learning progression for teams new to FTC programming who want to eventually work towards advanced autonomous navigation like Pedro Pathing.

### Phase 1: FTC Fundamentals (Weeks 1-4)

Start with the basics before attempting advanced frameworks. **Current focus for our team.**

#### Essential Concepts
1. **OpMode Structure**
   - Understanding LinearOpMode vs OpMode
   - The init-start-loop lifecycle
   - Using telemetry for debugging

2. **Hardware Basics**
   - HardwareMap configuration
   - Motor control and direction
   - Servo positioning
   - Gamepad input handling

#### Recommended Resources
- **Official FTC Docs - Getting Started**: https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-(Android-Studio).html
- **FTC Sample OpModes**: Study `BasicOpMode_Linear.java` and `BasicOpMode_Iterative.java` in SDK
- **Game Manual 0**: https://gm0.org/en/latest/ (comprehensive beginner guide)
- **FIRST Tech Challenge YouTube**: https://www.youtube.com/c/FIRSTTechChallenge (official tutorials)

#### Practice Projects
```java
// Week 1-2: Master these basics
1. Create a simple TeleOp drive program
2. Add telemetry to display motor positions
3. Control a servo with gamepad buttons
4. Display sensor values (distance, color)

// Week 3-4: Build on fundamentals
5. Create a simple autonomous (drive forward, turn, stop)
6. Use encoders to drive specific distances
7. Implement basic if/else logic for sensors
```

### Phase 2: Intermediate Programming (Weeks 5-8)

Build more sophisticated control systems and understand mathematical concepts.

#### Essential Concepts
1. **PID Control Basics**
   - Proportional control (P)
   - Integral and Derivative (I and D)
   - Tuning PID constants

2. **State Machines**
   - Finite State Machine (FSM) pattern
   - Managing autonomous sequences
   - Non-blocking code design

3. **Encoders and Odometry**
   - Understanding encoder ticks
   - Converting ticks to distance
   - Dead wheel odometry basics

#### Recommended Resources
- **Game Manual 0 - Control Theory**: https://gm0.org/en/latest/docs/software/concepts/control-loops.html
- **PID Without a PhD**: https://www.ni.com/en-us/shop/labview/pid-theory-explained.html
- **FTC PID Tutorial Video**: Search "FTC PID control tutorial" on YouTube
- **State Machine Tutorial**: https://gm0.org/en/latest/docs/software/finite-state-machines.html

#### Practice Projects
```java
// Week 5-6: PID Control
1. Implement P-only motor speed control
2. Add I and D terms, tune constants
3. Create a "drive straight" PID controller
4. Implement "turn to heading" PID

// Week 7-8: State Machines
5. Convert a simple autonomous to use FSM
6. Create multi-stage autonomous with timer transitions
7. Add sensor-based state transitions
```

### Phase 3: Advanced Autonomous Prep (Weeks 9-12)

Prepare for frameworks like Pedro Pathing by understanding the underlying mathematics and patterns.

#### Essential Concepts
1. **Coordinate Systems**
   - Field-centric vs robot-centric
   - Position tracking (x, y, heading)
   - Rotation matrices and transformations

2. **Path Planning Basics**
   - Waypoint navigation
   - Velocity profiles
   - Smooth acceleration/deceleration

3. **Holonomic Drive Kinematics** (if applicable)
   - Mecanum wheel math
   - Inverse kinematics
   - Field-relative driving

#### Recommended Resources
- **Game Manual 0 - Odometry**: https://gm0.org/en/latest/docs/software/concepts/odometry.html
- **Mecanum Drive Tutorial**: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
- **Bézier Curve Primer**: https://javascript.info/bezier-curve (concepts apply to robotics)
- **FTC Coordinate Systems**: https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/vision_overview/vision-overview.html

#### Practice Projects
```java
// Week 9-10: Position Tracking
1. Implement basic encoder-based odometry
2. Create a "drive to position" function
3. Test position tracking accuracy
4. Add heading tracking with IMU

// Week 11-12: Path Following
5. Create waypoint-based autonomous
6. Implement smooth velocity ramping
7. Add position error correction
8. Test multi-waypoint navigation
```

### Phase 4: Pedro Pathing Mastery (Weeks 13+)

**Prerequisites**: Holonomic drive + localization hardware + completion of Phases 1-3

#### Essential Concepts
1. **Pedro Pathing Architecture**
   - Follower initialization
   - Path building with Bézier curves
   - Tuning methodology

2. **Advanced Localization**
   - Two-wheel odometry
   - Three-wheel odometry
   - OTOS/Pinpoint integration

3. **Competition Autonomous**
   - Multi-alliance paths
   - Dynamic decision making
   - Robust error handling

#### Recommended Resources
- **Pedro Pathing Official Tutorial**: https://pedropathing.com/docs/pathing
- **Pedro Pathing Setup Video**: https://www.youtube.com/watch?v=6v7QgzRhOwA
- **Pedro Pathing GitHub Examples**: https://github.com/Pedro-Pathing/PedroPathing/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
- **FTC Localization Guide**: https://gm0.org/en/latest/docs/software/concepts/odometry.html

#### Learning Sequence
```
Week 13-14: Setup and Basic Tuning
├─ Install Pedro Pathing library
├─ Configure Constants.java
├─ Run Localization Test
├─ Complete Forward/Lateral/Turn tuners
└─ Verify position tracking accuracy

Week 15-16: Path Following
├─ Run Line/Triangle/Circle test paths
├─ Tune PIDF values for smooth following
├─ Complete velocity tuners
└─ Test path interruption and switching

Week 17-18: Autonomous Development
├─ Map field positions for your game strategy
├─ Build multi-segment autonomous paths
├─ Implement FSM for scoring sequences
├─ Add subsystem integration (arm, intake, etc.)
└─ Practice and iterate
```

### Phase 5: Ongoing Learning (Season-Long)

Advanced topics to explore as you gain experience.

#### Topics for Continuous Improvement
1. **Vision Processing**
   - AprilTag detection for localization
   - Color-based object detection
   - TensorFlow object recognition

2. **Advanced Architecture**
   - Command-based programming (BunyipsFTC)
   - Subsystem abstraction patterns
   - Reusable component libraries

3. **Competition Strategies**
   - Alliance-specific paths
   - Dynamic autonomous selection
   - Recovery from errors
   - Optimal scoring sequences

#### Recommended Resources
- **FTC Vision Portal**: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_overview/visionportal-overview.html
- **EasyOpenCV Tutorial**: https://github.com/OpenFTC/EasyOpenCV
- **BunyipsFTC Implementation**: /Users/jameswang/projects/BunyipsFTC (local reference)
- **FTC Forums**: https://ftc-community.firstinspires.org/ (ask questions, get help)

### Learning Resources by Category

#### Video Tutorials
- **FTC Official Programming Series**: https://www.youtube.com/@FIRSTTechChallenge
- **Wizards.exe Programming**: https://www.youtube.com/@wizards.exe (high-quality FTC tutorials)
- **CTRL ALT FTC**: https://www.youtube.com/@CTRLALTFTC (Pedro Pathing tutorials)

#### Written Guides
- **Game Manual 0**: https://gm0.org (THE comprehensive FTC resource)
- **FTC Docs**: https://ftc-docs.firstinspires.org (official documentation)
- **LearnRoadRunner**: https://learnroadrunner.com (path planning concepts)

#### Interactive Learning
- **FTC Blocks/OnBot Java**: Practice in browser without Android Studio
- **RoadRunner GUI**: https://gui.learnroadrunner.com (visual path planning)
- **FTC Simulator**: https://github.com/Virtual-Robot/ftc_app (test code virtually)

#### Community Support
- **FTC Discord**: https://discord.gg/first-tech-challenge
- **Chief Delphi FTC**: https://www.chiefdelphi.com/c/first-programs/first-tech-challenge/179
- **Reddit r/FTC**: https://reddit.com/r/FTC
- **GitHub Discussions**: Search for specific libraries (Pedro Pathing, FTCLib, etc.)

### Recommended Learning Path Timeline

```
Complete Beginner → Pedro Pathing Proficiency

Month 1-2: FTC Fundamentals
├─ Learn OpMode structure
├─ Master basic hardware control
├─ Create simple TeleOp and Autonomous
└─ Study sample code extensively

Month 3-4: Intermediate Concepts
├─ Implement PID control
├─ Learn state machine pattern
├─ Understand encoder-based navigation
└─ Build multi-stage autonomous

Month 5-6: Advanced Preparation
├─ Study coordinate systems
├─ Implement basic odometry
├─ Learn path planning concepts
└─ (ONLY if upgrading to mecanum drive)

Month 7+: Pedro Pathing (if applicable)
├─ Complete all tuning OpModes
├─ Build competition autonomous
├─ Integrate with subsystems
└─ Iterate and optimize

Note: Our team is currently at Month 1-2 with tank drive.
      Pedro Pathing requires mecanum drive upgrade first.
```

### Study Tips for Beginner Teams

1. **Start Small**: Don't try to learn everything at once. Master one concept before moving to the next.

2. **Read Sample Code**: The FTC SDK includes 65+ sample OpModes. Read them carefully and understand each line.

3. **Ask Questions**: Use Discord, forums, and team mentors. The FTC community is very helpful.

4. **Test Frequently**: Write small test programs to verify your understanding before building complex systems.

5. **Document Learning**: Keep notes on what works, what doesn't, and why. Build your team's knowledge base.

6. **Watch Competitive Teams**: Study GitHub repositories of successful teams to see real-world implementations.

7. **Attend Workshops**: Look for FTC workshops, scrimmages, and clinics in your area.

### Common Beginner Mistakes to Avoid

❌ **Jumping to advanced frameworks too early** → Master basics first
❌ **Copying code without understanding** → Study and comprehend before copying
❌ **Ignoring tuning and calibration** → Proper tuning is essential for reliability
❌ **Not testing incrementally** → Test each small change before adding more
❌ **Skipping documentation** → Read official docs thoroughly
❌ **Working in isolation** → Engage with the community for help and ideas

### Key Milestones for Beginner Teams

✅ **Milestone 1**: Successfully run a sample OpMode from the SDK
✅ **Milestone 2**: Create your first custom TeleOp with gamepad control
✅ **Milestone 3**: Drive forward 24 inches using encoder counts
✅ **Milestone 4**: Implement a simple PID controller
✅ **Milestone 5**: Create a 3-state autonomous using FSM pattern
✅ **Milestone 6**: Successfully complete a full autonomous routine in competition
✅ **Milestone 7**: (Advanced) Implement Pedro Pathing with tuned parameters

---

## Appendix: Quick Reference

### Pedro Pathing Quick Commands

```java
// Create follower
Follower follower = Constants.createFollower(hardwareMap);

// Set starting position
follower.setStartingPose(new Pose(0, 0, 0));

// Create straight path
Path path = new Path(new BezierLine(start, end));

// Create curved path
Path curve = new Path(new BezierCurve(start, control, end));

// Follow path
follower.followPath(path);

// Update (call every loop)
follower.update();

// Check if busy
boolean moving = follower.isBusy();

// Get current position
Pose currentPose = follower.getPose();

// Teleop drive
follower.setTeleOpDrive(forward, strafe, turn, fieldRelative);
```

### Coordinate System Cheat Sheet

```
FTC Field (144" x 144")

Red Alliance View:
  Y ↑
    |
    |___→ X

Heading:
- 0° = facing +X (right)
- 90° = facing +Y (forward)
- 180° = facing -X (left)
- 270° = facing -Y (backward)

Convert degrees to radians:
Math.toRadians(degrees)
```

### Troubleshooting Checklist

**Localization not working:**
- [ ] Encoder wires properly connected?
- [ ] Encoder directions correct in Constants?
- [ ] IMU orientation correct?
- [ ] Ran localization tuners?

**Robot not following path:**
- [ ] Called `follower.update()` in loop?
- [ ] PIDF values tuned?
- [ ] Path constraints reasonable?
- [ ] Velocity measurements completed?

**Robot overshooting paths:**
- [ ] Zero power acceleration tuned?
- [ ] Path constraints too aggressive?
- [ ] Centripetal correction tuned?

**Robot drifting:**
- [ ] Localization multipliers accurate?
- [ ] Wheels slipping on field?
- [ ] Battery voltage consistent?

---

## Conclusion

Brighton-FTC's implementation demonstrates the power of **specialized, purpose-built frameworks** for autonomous navigation. Pedro Pathing represents a significant evolution beyond basic FTC path following, offering:

- **Mathematical precision** through Bézier curves
- **Real-time adaptation** with multi-vector control
- **Comprehensive tuning** through 15+ specialized OpModes
- **Visual debugging** with Panels Field integration

**However**, it comes with clear trade-offs:
- ❌ Requires holonomic drive (mecanum/omni wheels)
- ❌ Requires localization hardware
- ❌ Requires Android Studio
- ❌ Focused primarily on autonomous (not TeleOp enhancements)

**For our team** with a 2-motor tank drive, Pedro Pathing is **not applicable** this season. However, understanding these patterns prepares us for potential future upgrades and demonstrates the sophisticated approaches used by competitive FTC teams.

**Key Takeaway**: Different frameworks excel at different goals. Standard FTC teaches fundamentals, BunyipsFTC provides professional architecture, and Pedro Pathing delivers autonomous excellence. Choose the tool that matches your robot's capabilities and competition goals.

---

**Document Version**: 1.0
**Last Updated**: November 9, 2025
**Analyzed By**: Claude Code (FTC SDK 11.0)
**Repository**: Brighton-FTC 2026 Season Implementation
