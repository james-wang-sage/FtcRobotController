# AprilTag Vision Guide

> **Use camera vision to detect AprilTags for navigation, alignment, and field localization.**

## Table of Contents

1. [What Are AprilTags?](#what-are-apriltags)
2. [DECODE Season AprilTag IDs](#decode-season-apriltag-ids)
3. [Data From AprilTag Detection](#data-from-apriltag-detection)
4. [Understanding ftcPose (Camera-Relative)](#understanding-ftcpose-camera-relative)
5. [Understanding robotPose (Field Localization)](#understanding-robotpose-field-localization)
6. [Configuring Camera Pose](#configuring-camera-pose)
7. [Code Examples](#code-examples)
8. [Practical Use Cases](#practical-use-cases)
9. [Troubleshooting](#troubleshooting)
10. [Quick Reference](#quick-reference)
11. [Learning Path](#learning-path)

---

## What Are AprilTags?

AprilTags are **2D barcodes** (fiducial markers) that robots can detect with a camera. Each tag has a unique ID and known physical size, allowing the vision system to calculate:

- **Tag identity** (which tag is it?)
- **Tag position** (where is it relative to the camera?)
- **Tag orientation** (how is it rotated?)

```
┌─────────────────────────────────────────────────────────────────┐
│                    APRILTAG DETECTION FLOW                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌──────────┐      ┌───────────────┐      ┌────────────────┐   │
│   │  Camera  │ ───▶ │ AprilTag      │ ───▶ │ Detection Data │   │
│   │  Image   │      │ Processor     │      │ (ID, Pose)     │   │
│   └──────────┘      └───────────────┘      └────────────────┘   │
│                                                                 │
│   What it sees:           What it does:        What you get:    │
│   ┌─────────────┐         Identifies tags      • Tag ID (24)    │
│   │ ▓▓░░▓▓░░▓▓  │         Calculates pose      • Position XYZ   │
│   │ ░░▓▓░░▓▓░░  │         Measures distance    • Orientation    │
│   │ ▓▓░░▓▓░░▓▓  │         Tracks corners       • Range/Bearing  │
│   │ ░░▓▓░░▓▓░░  │                                               │
│   └─────────────┘                                               │
│     AprilTag                                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Why AprilTags?

| Advantage | Description |
|-----------|-------------|
| **Unique IDs** | Each tag has a distinct number — no confusion |
| **3D Pose** | Get full position AND orientation in one detection |
| **Fast** | Real-time detection at 30+ FPS |
| **Robust** | Works at angles, partial occlusion, varying light |
| **Known Positions** | Field tags are at documented locations |

---

## DECODE Season AprilTag IDs

The DECODE (2025-2026) field has **5 AprilTags**:

```
┌─────────────────────────────────────────────────────────────────┐
│                    DECODE FIELD APRILTAG LAYOUT                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                       TOP OF FIELD (+Y)                         │
│         ┌─────────────────────────────────────┐                 │
│         │                                     │                 │
│         │   BLUE GOAL              RED GOAL   │                 │
│         │   [Tag 20]               [Tag 24]   │                 │
│         │      ╲                     ╱        │                 │
│         │       ╲                   ╱         │                 │
│         │        ╲     OBELISK    ╱           │                 │
│         │         ╲   [21,22,23] ╱            │                 │
│         │          ╲     ◆     ╱              │                 │
│         │           ╲   ╱╲   ╱                │                 │
│         │            ╲╱  ╲╱                   │                 │
│         │                                     │                 │
│   (-X)  │                                     │  (+X)           │
│   BLUE  │            FIELD CENTER             │  RED            │
│   SIDE  │                                     │  SIDE           │
│         │                                     │                 │
│         └─────────────────────────────────────┘                 │
│                      AUDIENCE (-Y)                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Tag Reference Table

| Tag ID | Location | Use for Navigation? | Notes |
|--------|----------|---------------------|-------|
| **20** | Blue Alliance Goal | ✅ **YES** | Primary target for Blue alliance |
| **21** | Obelisk Face 1 | ❌ **NO** | Obelisk can rotate — unreliable position |
| **22** | Obelisk Face 2 | ❌ **NO** | Obelisk can rotate — unreliable position |
| **23** | Obelisk Face 3 | ❌ **NO** | Obelisk can rotate — unreliable position |
| **24** | Red Alliance Goal | ✅ **YES** | Primary target for Red alliance |

### Code Constants (from DecodeField.java)

```java
// Goal tags — USE THESE for navigation
public static final int APRILTAG_RED_GOAL = 24;
public static final int APRILTAG_BLUE_GOAL = 20;

// Obelisk tags — DO NOT use for localization
public static final int APRILTAG_OBELISK_FACE_1 = 21;
public static final int APRILTAG_OBELISK_FACE_2 = 22;
public static final int APRILTAG_OBELISK_FACE_3 = 23;
```

### Helper Methods

```java
// Check if a tag is on a goal (safe for navigation)
DecodeField.isGoalTag(24)      // → true
DecodeField.isGoalTag(22)      // → false

// Check if a tag is on the obelisk (avoid for navigation)
DecodeField.isObeliskTag(22)   // → true
DecodeField.isObeliskTag(24)   // → false

// Get goal tag for your alliance
DecodeField.getGoalAprilTagId(Alliance.RED)   // → 24
DecodeField.getGoalAprilTagId(Alliance.BLUE)  // → 20
```

---

## Data From AprilTag Detection

When an AprilTag is detected, the `AprilTagDetection` object provides extensive information:

```
┌─────────────────────────────────────────────────────────────────┐
│                 APRILTAG DETECTION DATA STRUCTURE               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   AprilTagDetection                                             │
│   ├── id                    → Tag number (e.g., 24)             │
│   ├── metadata                                                  │
│   │   └── name              → Human name (e.g., "RedGoal")      │
│   ├── center                                                    │
│   │   ├── x                 → Pixel X in camera image           │
│   │   └── y                 → Pixel Y in camera image           │
│   ├── ftcPose               → Position RELATIVE TO CAMERA       │
│   │   ├── x, y, z           → 3D position (inches)              │
│   │   ├── pitch, roll, yaw  → 3D orientation (degrees)          │
│   │   ├── range             → Direct distance (inches)          │
│   │   ├── bearing           → Horizontal angle (degrees)        │
│   │   └── elevation         → Vertical angle (degrees)          │
│   ├── robotPose             → Position RELATIVE TO FIELD        │
│   │   ├── position          → Robot X, Y, Z on field            │
│   │   └── orientation       → Robot heading on field            │
│   └── decisionMargin        → Detection confidence              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Two Coordinate Systems

| Property | Reference Frame | Use Case |
|----------|-----------------|----------|
| `ftcPose` | Camera-centric | "Drive toward the tag" |
| `robotPose` | Field-centric | "Where am I on the field?" |

---

## Understanding ftcPose (Camera-Relative)

`ftcPose` tells you where the tag is **relative to your camera**. This is ideal for driving toward targets.

### ftcPose Coordinate System

```
┌─────────────────────────────────────────────────────────────────┐
│                    ftcPose COORDINATE SYSTEM                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                        (Camera's View)                          │
│                                                                 │
│                           +Z (up)                               │
│                            ▲                                    │
│                            │                                    │
│                            │                                    │
│              ◄─────────────┼─────────────►                      │
│           -X (left)        │           +X (right)               │
│                            │                                    │
│                            │                                    │
│                            ╳ ─ ─ ─ ─ ─ ─ ▶ +Y (forward)         │
│                         Camera                                  │
│                                                                 │
│   If tag is detected at ftcPose (x=5, y=24, z=3):               │
│   • 5 inches to the RIGHT of camera center                      │
│   • 24 inches FORWARD (away from camera)                        │
│   • 3 inches UP from camera level                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### ftcPose Properties

| Property | Description | Units | Example |
|----------|-------------|-------|---------|
| `ftcPose.x` | Right (+) / Left (-) | inches | `5.2` = 5.2" right of center |
| `ftcPose.y` | Forward (+) / Back (-) | inches | `24.0` = 24" ahead |
| `ftcPose.z` | Up (+) / Down (-) | inches | `3.1` = 3.1" above camera |
| `ftcPose.range` | Direct distance to tag | inches | `24.6` = straight-line distance |
| `ftcPose.bearing` | Horizontal angle | degrees | `12.2` = 12.2° right of center |
| `ftcPose.elevation` | Vertical angle | degrees | `7.4` = 7.4° above horizon |
| `ftcPose.pitch` | Tag rotation (X-axis) | degrees | Tag tilted forward/back |
| `ftcPose.roll` | Tag rotation (Z-axis) | degrees | Tag tilted sideways |
| `ftcPose.yaw` | Tag rotation (Y-axis) | degrees | Tag rotated left/right |

### Cartesian vs Polar

```
┌─────────────────────────────────────────────────────────────────┐
│                TWO WAYS TO DESCRIBE POSITION                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   CARTESIAN (XYZ):                  POLAR (Range/Bearing):      │
│   ─────────────────                 ────────────────────        │
│                                                                 │
│         Tag ●                              Tag ●                │
│            ╱│                                 ╲                 │
│           ╱ │ y=24"                            ╲ range=24.6"    │
│          ╱  │                                   ╲               │
│         ╱   │                         bearing=12°╲              │
│   x=5" ╱    │                                     ╲             │
│       ╱     │                                      ╲            │
│      ●──────┘                                       ●           │
│   Camera                                         Camera         │
│                                                                 │
│   "5 inches right,              "24.6 inches away,              │
│    24 inches forward"            12 degrees right"              │
│                                                                 │
│   Use for: precise positioning   Use for: simple navigation     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Code Example: Using ftcPose

```java
// Drive toward a detected tag
for (AprilTagDetection detection : aprilTag.getDetections()) {
    if (detection.id == DecodeField.APRILTAG_RED_GOAL) {
        // Option 1: Use range and bearing (simpler)
        double distanceToTag = detection.ftcPose.range;      // inches
        double angleToTag = detection.ftcPose.bearing;       // degrees

        telemetry.addData("Distance", "%.1f inches", distanceToTag);
        telemetry.addData("Bearing", "%.1f degrees", angleToTag);

        // Drive logic: rotate until bearing ≈ 0, then drive forward
        if (Math.abs(angleToTag) > 3.0) {
            // Rotate toward tag
            double turnPower = angleToTag > 0 ? 0.2 : -0.2;
        } else if (distanceToTag > 12.0) {
            // Drive forward
            double drivePower = 0.3;
        }

        // Option 2: Use XYZ (more precise)
        double rightOffset = detection.ftcPose.x;   // + = tag is right
        double forwardDist = detection.ftcPose.y;   // + = tag is ahead
    }
}
```

---

## Understanding robotPose (Field Localization)

`robotPose` answers the question: **"Where am I on the field?"**

### The Key Difference

```
┌─────────────────────────────────────────────────────────────────┐
│              ftcPose vs robotPose COMPARISON                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ftcPose (Camera-Relative):                                    │
│   ─────────────────────────                                     │
│   "Tag 24 is 24 inches forward and 5 inches right of camera"    │
│                                                                 │
│   Good for: Driving toward the tag                              │
│   Limitation: Doesn't tell you WHERE you are on the field       │
│                                                                 │
│   ─────────────────────────────────────────────────────────     │
│                                                                 │
│   robotPose (Field-Relative):                                   │
│   ────────────────────────────                                  │
│   "Robot is at field position (1200, 800) facing 45 degrees"    │
│                                                                 │
│   Good for: Knowing your location, planning paths               │
│   Requirement: Must configure camera pose first                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### How robotPose Works

```
┌─────────────────────────────────────────────────────────────────┐
│                 robotPose CALCULATION FLOW                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   INPUTS:                                                       │
│   ┌─────────────────┐                                           │
│   │ 1. Tag's known  │  From tag library (e.g., Tag 24 is at     │
│   │    field pos    │  position X=1700mm, Y=1700mm on field)    │
│   └────────┬────────┘                                           │
│            │                                                    │
│            ▼                                                    │
│   ┌─────────────────┐                                           │
│   │ 2. Camera-to-   │  ftcPose gives relative position          │
│   │    tag offset   │  (tag is 24" forward, 5" right)           │
│   └────────┬────────┘                                           │
│            │                                                    │
│            ▼                                                    │
│   ┌─────────────────┐                                           │
│   │ 3. Camera pose  │  YOU configure where camera is mounted    │
│   │    on robot     │  (camera is 8" forward, 6" up on robot)   │
│   └────────┬────────┘                                           │
│            │                                                    │
│            ▼                                                    │
│   OUTPUT:                                                       │
│   ┌─────────────────┐                                           │
│   │ Robot's field   │  "Robot is at (1200, 800) heading 45°"    │
│   │ position!       │                                           │
│   └─────────────────┘                                           │
│                                                                 │
│   SDK does the math:                                            │
│   robot_pos = tag_pos - camera_to_tag - camera_offset_on_robot  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Visual Example

```
┌────────────────────────────────────────────────────────────────┐
│                      FTC FIELD (144" × 144")                   │
│                                                                │
│   Blue Goal                                    Red Goal        │
│   (Tag 20)                                     (Tag 24)        │
│      ■                                             ■           │
│                                                                │
│                          🤖 Robot                              │
│                       position: (72, 48)                       │
│                       heading: 45°                             │
│                                                                │
│   ─────────────────────────────────────────────────────────    │
│   Origin (0,0)                                                 │
└────────────────────────────────────────────────────────────────┘

ftcPose says: "Tag 24 is 60 inches away at bearing 30°"
robotPose says: "Robot is at field coordinates (72, 48), heading 45°"
```

### robotPose Properties

| Property | Description | Access |
|----------|-------------|--------|
| **Position X** | Robot's field X coordinate | `detection.robotPose.getPosition().x` |
| **Position Y** | Robot's field Y coordinate | `detection.robotPose.getPosition().y` |
| **Position Z** | Robot's field Z coordinate | `detection.robotPose.getPosition().z` |
| **Yaw** | Robot's heading on field | `detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)` |
| **Pitch** | Robot's forward tilt | `detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES)` |
| **Roll** | Robot's side tilt | `detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES)` |

---

## Configuring Camera Pose

**CRITICAL:** `robotPose` will be `null` unless you configure the camera pose!

### Step 1: Measure Camera Position on Robot

```
┌─────────────────────────────────────────────────────────────────┐
│                MEASURING CAMERA POSITION                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   TOP VIEW OF ROBOT:                  SIDE VIEW:                │
│   ─────────────────                   ───────────               │
│                                                                 │
│        FRONT                              ┌─Camera              │
│          ▲                                │  │                  │
│          │                                │  │ z = 12"          │
│   ┌──────┴──────┐                         │  │ (height)         │
│   │             │                     ────┴──┴────              │
│   │     ◯       │ ← Camera               Robot                  │
│   │   (8",0")   │   at x=8" forward                             │
│   │             │   y=0" (centered)                             │
│   │      ●      │ ← Robot center                                │
│   │   (0",0")   │                                               │
│   └─────────────┘                                               │
│                                                                 │
│   Camera Position = (x=8, y=0, z=12) inches                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Step 2: Determine Camera Orientation

```
┌─────────────────────────────────────────────────────────────────┐
│                 CAMERA ORIENTATION                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   DEFAULT (all zeros): Camera pointing STRAIGHT UP              │
│   ─────────────────────────────────────────────                 │
│                                                                 │
│   MOST COMMON SETUP: Camera pointing FORWARD (horizontal)       │
│   ─────────────────────────────────────────────────────         │
│   Pitch = -90°  (rotated down from vertical to horizontal)      │
│   Yaw = 0°      (pointing straight ahead)                       │
│   Roll = 0°     (not tilted sideways)                           │
│                                                                 │
│   ANGLED CAMERA: Camera tilted down 15° from horizontal         │
│   ────────────────────────────────────────────────────          │
│   Pitch = -90° - 15° = -105°                                    │
│                                                                 │
│   SIDE-MOUNTED: Camera pointing left                            │
│   ─────────────────────────────────                             │
│   Pitch = -90°                                                  │
│   Yaw = +90°    (pointing left instead of forward)              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Step 3: Configure in Code

```java
// Define camera mounting position (relative to robot center)
private Position cameraPosition = new Position(
    DistanceUnit.INCH,
    8.0,    // x: 8 inches FORWARD from robot center
    0.0,    // y: 0 inches LEFT (centered)
    12.0,   // z: 12 inches UP from ground
    0       // acquisitionTime (can be 0)
);

// Define camera orientation
private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
    AngleUnit.DEGREES,
    0,      // yaw: 0° = pointing forward
    -90,    // pitch: -90° = horizontal (not pointing up)
    0,      // roll: 0° = not tilted sideways
    0       // acquisitionTime (can be 0)
);

// Build AprilTag processor WITH camera pose
aprilTag = new AprilTagProcessor.Builder()
    .setCameraPose(cameraPosition, cameraOrientation)  // ← REQUIRED!
    .build();
```

### Common Camera Configurations

| Mounting Style | Position (x, y, z) | Orientation (yaw, pitch, roll) |
|----------------|-------------------|--------------------------------|
| **Front center, 12" high** | (8, 0, 12) | (0, -90, 0) |
| **Left side, forward-facing** | (0, 6, 10) | (0, -90, 0) |
| **Right side, forward-facing** | (0, -6, 10) | (0, -90, 0) |
| **Front, tilted down 15°** | (8, 0, 14) | (0, -105, 0) |
| **Side-mounted, looking left** | (0, 6, 10) | (90, -90, 0) |

---

## Code Examples

### Basic Detection (ConceptAprilTagEasy)

```java
@TeleOp(name = "AprilTag Easy", group = "Concept")
public class AprilTagEasy extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize with defaults (no robotPose)
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("Tag %d: %s",
                        detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("  Range: %.1f inches",
                        detection.ftcPose.range));
                    telemetry.addLine(String.format("  Bearing: %.1f degrees",
                        detection.ftcPose.bearing));
                }
            }
            telemetry.update();
            sleep(20);
        }
    }
}
```

### With Field Localization (robotPose)

```java
@TeleOp(name = "AprilTag Localization", group = "Concept")
public class AprilTagLocalization extends LinearOpMode {

    // Camera mounting configuration
    private Position cameraPosition = new Position(DistanceUnit.INCH,
        8, 0, 12, 0);  // 8" forward, 12" high
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
        AngleUnit.DEGREES, 0, -90, 0, 0);  // Horizontal, facing forward

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize WITH camera pose for robotPose
        aprilTag = new AprilTagProcessor.Builder()
            .setCameraPose(cameraPosition, cameraOrientation)  // ← Enables robotPose!
            .build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();

        waitForStart();

        while (opModeIsActive()) {
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                // Only use goal tags for localization (not obelisk)
                if (DecodeField.isGoalTag(detection.id) && detection.robotPose != null) {

                    // Get robot's field position
                    double robotX = detection.robotPose.getPosition().x;
                    double robotY = detection.robotPose.getPosition().y;
                    double robotHeading = detection.robotPose.getOrientation()
                        .getYaw(AngleUnit.DEGREES);

                    telemetry.addLine("=== ROBOT FIELD POSITION ===");
                    telemetry.addData("X", "%.1f inches", robotX);
                    telemetry.addData("Y", "%.1f inches", robotY);
                    telemetry.addData("Heading", "%.1f degrees", robotHeading);
                }
            }
            telemetry.update();
            sleep(20);
        }
    }
}
```

### Using AprilTagLocalizer (Team Code)

```java
// In your OpMode init
AprilTagLocalizer localizer = new AprilTagLocalizer(hardwareMap, "Webcam 1");
localizer.setCameraPosition(200, 0, 150);  // mm: 200 forward, 150 up
localizer.setCameraOrientation(Math.toRadians(-90), 0);  // Horizontal

// In your loop
Pose2d robotPose = localizer.getEstimatedPose(Alliance.RED);
if (robotPose != null) {
    telemetry.addData("Position", "(%.0f, %.0f) mm",
        robotPose.getX(), robotPose.getY());
    telemetry.addData("Heading", "%.1f degrees",
        Math.toDegrees(robotPose.getHeading()));
}
```

---

## Practical Use Cases

### Use Case 1: Drive to Goal (ftcPose)

```java
// Align robot to face Tag 24 (Red Goal)
AprilTagDetection goalTag = findTag(24);
if (goalTag != null) {
    double bearing = goalTag.ftcPose.bearing;

    if (Math.abs(bearing) > 2.0) {
        // Rotate to face the tag
        double turnPower = -bearing * 0.02;  // P-control
        setMotorPowers(turnPower, -turnPower, turnPower, -turnPower);
    } else {
        // Aligned! Drive forward
        double range = goalTag.ftcPose.range;
        if (range > 18.0) {  // Stop 18" from goal
            setMotorPowers(0.3, 0.3, 0.3, 0.3);
        }
    }
}
```

### Use Case 2: Know Your Position (robotPose)

```java
// Autonomous: Navigate to specific field position
AprilTagDetection detection = getBestGoalDetection();
if (detection != null && detection.robotPose != null) {
    double currentX = detection.robotPose.getPosition().x;
    double currentY = detection.robotPose.getPosition().y;

    // Target position (e.g., loading zone)
    double targetX = -48;
    double targetY = -60;

    double errorX = targetX - currentX;
    double errorY = targetY - currentY;
    double distance = Math.sqrt(errorX*errorX + errorY*errorY);

    telemetry.addData("Distance to target", "%.1f inches", distance);
}
```

### Use Case 3: Sensor Fusion (Odometry + AprilTag)

```java
// Correct odometry drift with AprilTag when visible
Pose2d odometryPose = odometry.getPose();
Pose2d visionPose = localizer.getEstimatedPose(alliance);

if (visionPose != null && localizer.hasRecentDetection(500)) {
    // Blend odometry with vision (70% odometry, 30% vision)
    double blendedX = odometryPose.getX() * 0.7 + visionPose.getX() * 0.3;
    double blendedY = odometryPose.getY() * 0.7 + visionPose.getY() * 0.3;

    odometry.setPose(new Pose2d(blendedX, blendedY, odometryPose.getHeading()));
}
```

---

## Troubleshooting

### Problem: robotPose is always null

**Cause:** Camera pose not configured

**Fix:** Add `.setCameraPose()` when building the processor:
```java
aprilTag = new AprilTagProcessor.Builder()
    .setCameraPose(cameraPosition, cameraOrientation)  // ← ADD THIS
    .build();
```

---

### Problem: Tag detected but metadata is null

**Cause:** Tag ID not in the tag library

**Fix:** Use the current game tag library:
```java
.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
```

---

### Problem: Position values seem wrong

**Possible causes:**
1. Camera position/orientation configured incorrectly
2. Units mismatch (inches vs mm)
3. Camera mounted differently than configured

**Fix:**
1. Measure camera position carefully
2. Check units in `Position` constructor
3. Verify pitch = -90° for horizontal camera

---

### Problem: Low detection rate (tags not detected)

**Possible causes:**
1. Camera resolution too low
2. Decimation too high
3. Poor lighting
4. Camera out of focus

**Fix:**
```java
// Increase resolution
builder.setCameraResolution(new Size(1280, 720));

// Lower decimation for longer range (slower but better detection)
aprilTag.setDecimation(2);  // Default is 3
```

---

### Problem: Robot position jumps around

**Cause:** Camera vibration or noisy detections

**Fix:** Add filtering:
```java
// Only use high-confidence detections
if (detection.decisionMargin > 15.0) {
    // Use this detection
}

// Average multiple frames
// (implement a rolling average of positions)
```

---

## Quick Reference

### ftcPose vs robotPose Summary

| Aspect | `ftcPose` | `robotPose` |
|--------|-----------|-------------|
| **Reference** | Camera | Field |
| **Question answered** | "Where is the tag?" | "Where am I?" |
| **Requires camera config** | No | Yes |
| **Best for** | Driving toward targets | Navigation planning |
| **Always available** | Yes (if tag in library) | Only with camera pose |

### DECODE AprilTag Quick Reference

| Tag | Location | Use for Nav? | Constant |
|-----|----------|--------------|----------|
| 20 | Blue Goal | ✅ Yes | `APRILTAG_BLUE_GOAL` |
| 21 | Obelisk | ❌ No | `APRILTAG_OBELISK_FACE_1` |
| 22 | Obelisk | ❌ No | `APRILTAG_OBELISK_FACE_2` |
| 23 | Obelisk | ❌ No | `APRILTAG_OBELISK_FACE_3` |
| 24 | Red Goal | ✅ Yes | `APRILTAG_RED_GOAL` |

### Common Code Patterns

```java
// Get all detections
List<AprilTagDetection> detections = aprilTag.getDetections();

// Find specific tag
AprilTagDetection tag24 = detections.stream()
    .filter(d -> d.id == 24)
    .findFirst().orElse(null);

// Check if goal tag
if (DecodeField.isGoalTag(detection.id)) { ... }

// Get range and bearing (for driving toward)
double range = detection.ftcPose.range;
double bearing = detection.ftcPose.bearing;

// Get field position (requires camera pose config)
double fieldX = detection.robotPose.getPosition().x;
double fieldY = detection.robotPose.getPosition().y;
double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
```

---

## Learning Path

### Beginner Resources

| Resource | Link | Description |
|----------|------|-------------|
| **FTC AprilTag Introduction** | [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html) | Official getting started guide |
| **Detection Values Explained** | [ftc-docs: Detection Values](https://ftc-docs.firstinspires.org/apriltag-detection-values) | Understanding ftcPose values |
| **Field Coordinate System** | [ftc-docs: Field Coordinates](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html) | FTC field coordinate conventions |

### Sample OpModes to Study

| Sample | Location | What It Teaches |
|--------|----------|-----------------|
| `ConceptAprilTagEasy` | SDK Samples | Minimal setup |
| `ConceptAprilTag` | SDK Samples | Full configuration options |
| `ConceptAprilTagLocalization` | SDK Samples | Using robotPose |
| `RobotAutoDriveToAprilTagOmni` | SDK Samples | Driving toward tags |
| `AprilTagLocalizer` | TeamCode/pickle | Team implementation |

### Progression Path

```
┌─────────────────────────────────────────────────────────────────┐
│                    APRILTAG LEARNING PATH                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   LEVEL 1: Basic Detection                                      │
│   ─────────────────────────                                     │
│   □ Run ConceptAprilTagEasy                                     │
│   □ Display tag ID and name                                     │
│   □ Show range and bearing                                      │
│                                                                 │
│   LEVEL 2: Navigation                                           │
│   ────────────────────                                          │
│   □ Drive robot toward a tag                                    │
│   □ Stop at specific distance                                   │
│   □ Align to face tag directly                                  │
│                                                                 │
│   LEVEL 3: Localization                                         │
│   ──────────────────────                                        │
│   □ Configure camera pose                                       │
│   □ Get robotPose field coordinates                             │
│   □ Display robot position on field                             │
│                                                                 │
│   LEVEL 4: Sensor Fusion                                        │
│   ──────────────────────                                        │
│   □ Combine with odometry                                       │
│   □ Correct drift when tags visible                             │
│   □ Handle tag visibility loss gracefully                       │
│                                                                 │
│   LEVEL 5: Advanced                                             │
│   ─────────────────                                             │
│   □ Multi-tag averaging                                         │
│   □ Confidence-weighted fusion                                  │
│   □ Path planning with position feedback                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Related Documentation

- [L1-AutoAlign-Guide.md](L1-AutoAlign-Guide.md) — Using AprilTags for goal alignment
- [AutoMode-Guide-Holonomic.md](AutoMode-Guide-Holonomic.md) — Autonomous navigation
- [Mecanum-Drive-Guide.md](Mecanum-Drive-Guide.md) — Holonomic drive basics
- [IMU-TeleOp-Guide.md](IMU-TeleOp-Guide.md) — Combining IMU with vision

---

## Team Code Files

| File | Purpose |
|------|---------|
| `pickle/vision/AprilTagLocalizer.java` | Team's localization implementation |
| `pickle/field/DecodeField.java` | DECODE field constants and tag IDs |
| `concept/ConceptAprilTagEasy.java` | Simple detection example |
