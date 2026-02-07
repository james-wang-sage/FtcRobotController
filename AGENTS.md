# AGENTS.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

FIRST Tech Challenge (FTC) Robot Controller SDK v11.0 for the DECODE (2025-2026) competition season. This is an Android application that runs on the Robot Controller phone/Control Hub to control competition robots.

## Build Commands

```bash
# Build all modules
./gradlew build

# Build and install to connected robot controller device
./gradlew installDebug

# Run tests
./gradlew test

# Clean build artifacts
./gradlew clean
```

**Requirements:** JDK 21, Android SDK 30 (API Level 30)

**Note:** Source compatibility is set to Java 8 (`sourceCompatibility JavaVersion.VERSION_1_8`) for Android runtime compatibility.

## Project Structure

Two Gradle modules defined in `settings.gradle`:
- **FtcRobotController/** - Core SDK library (avoid modifying)
- **TeamCode/** - Team-specific robot code (primary development location)

### TeamCode Organization

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── auto/           # Autonomous driving samples (encoder, time, line, AprilTag)
├── basic/          # Minimal OpMode examples (Iterative, Linear, Omni)
├── starter/        # StarterBot examples (tank/skid-steer drive)
├── concept/        # Vision demonstrations (AprilTag)
├── sensor/         # Sensor usage examples
└── pickle/         # Advanced mecanum robot with modular architecture
    ├── config/     # Hardware name constants (PickleHardwareNames)
    ├── drive/      # Mecanum drive utilities
    ├── field/      # Field constants, alliance info
    ├── geometry/   # Pose2d, Translation2d, Rectangle2d
    ├── odometry/   # Position tracking
    ├── path/       # PathFollower, Waypoint for autonomous
    └── vision/     # AprilTag localization
```

## Key Architecture Patterns

### OpMode Types
- **LinearOpMode**: Sequential execution with `runOpMode()`, uses `waitForStart()`, suitable for simpler autonomous
- **OpMode**: Event-driven with `init()`, `init_loop()`, `start()`, `loop()`, `stop()` callbacks, preferred for state machines

### State Machine Pattern (Autonomous)
Used extensively for non-blocking autonomous routines. See `StarterBotAuto.java` for reference:
```java
private enum AutonomousState {
    LAUNCH,
    WAIT_FOR_LAUNCH,
    DRIVING_AWAY_FROM_GOAL,
    // ...
}
```

### Hardware Configuration
- Hardware names must match Driver Station robot configuration
- Use constants classes to avoid stringly-typed errors:
```java
// pickle/config/PickleHardwareNames.java
public static final String FRONT_LEFT_MOTOR = "front_left";
```

### Geometry Classes (`pickle/geometry/`)
- **Pose2d**: Position (x, y) + heading in field coordinates. Units: mm, radians
- **Translation2d**: 2D vector for positions/displacements
- Heading convention: 0 = +X direction, counter-clockwise positive

### Coordinate Systems
- **Field**: Origin at corner, 0° = +X, 90° = +Y (far wall), CCW positive
- **Robot/IMU**: 0° = heading at match start, CCW = positive angles

## Hardware Specifications

**Drive Motors**: goBILDA 5203 Series Yellow Jacket (312 RPM, 19.2:1 gear ratio, 537.7 CPR)
- Max velocity: ~2,796 ticks/sec at output shaft
- FTC SDK measures velocity at OUTPUT SHAFT (post-gearbox)

**Launcher Control**: Uses `RUN_USING_ENCODER` mode with custom PIDF coefficients for velocity control.

## Critical Conventions

1. **@Disabled Annotation**: New OpModes must have `@Disabled` removed to appear in Driver Station
2. **Motor Directions**: Configure `setDirection()` based on physical mounting - one side typically needs `REVERSE`
3. **Encoder Ports**: For `RUN_USING_ENCODER`, encoder must be plugged into port adjacent to motor
4. **Zero Power Behavior**: Use `BRAKE` for controllable stops on drive motors
5. **DistanceUnit/AngleUnit**: FTC SDK provides unit conversion utilities - use them for clarity

## Documentation Resources

- Team guides in `doc/guides/` covering drivetrain, programming, and vision
- Official FTC docs: https://ftc-docs.firstinspires.org/
- Javadoc: https://javadoc.io/doc/org.firstinspires.ftc
