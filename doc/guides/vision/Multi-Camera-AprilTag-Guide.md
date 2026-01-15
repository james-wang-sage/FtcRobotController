# Multi-Camera AprilTag Guide

> **Expand your robot's vision with multiple cameras for continuous localization, wider coverage, and improved accuracy.**

## Table of Contents

1. [Why Multiple Cameras?](#why-multiple-cameras)
2. [Single vs Multi-Camera Comparison](#single-vs-multi-camera-comparison)
3. [SDK Multi-Camera Modes](#sdk-multi-camera-modes)
4. [Camera Placement Strategies](#camera-placement-strategies)
5. [Configuring Multiple Camera Poses](#configuring-multiple-camera-poses)
6. [Fusion Algorithms](#fusion-algorithms)
7. [Code Examples](#code-examples)
8. [Practical Considerations](#practical-considerations)
9. [Troubleshooting](#troubleshooting)
10. [Quick Reference](#quick-reference)
11. [Learning Path](#learning-path)

---

## Why Multiple Cameras?

A single camera has a limited field of view (~60°), meaning your robot can only see AprilTags in one direction. Multiple cameras solve this limitation and provide several key advantages.

```
┌─────────────────────────────────────────────────────────────────┐
│           THE PROBLEM WITH A SINGLE CAMERA                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                    Tag 24 (Red Goal)                            │
│                         ■                                       │
│                         │                                       │
│                         │  "I can see this!"                    │
│                         ▼                                       │
│                    ╱ · · · · ╲                                  │
│                   ╱           ╲                                 │
│                  ╱   VISIBLE   ╲                                │
│                 ╱    (~60°)     ╲                               │
│                ╱                 ╲                               │
│   ════════════╱═══════════════════╲════════════                 │
│   ▓▓▓▓▓▓▓▓▓▓▓│       🤖          │▓▓▓▓▓▓▓▓▓▓▓                  │
│   ▓ BLIND ▓▓▓│      Robot        │▓▓▓ BLIND ▓                  │
│   ▓ SPOT  ▓▓▓│                   │▓▓▓ SPOT  ▓                  │
│   ▓▓▓▓▓▓▓▓▓▓▓└───────────────────┘▓▓▓▓▓▓▓▓▓▓▓                  │
│   ════════════════════════════════════════════                  │
│                         ▲                                       │
│                         │                                       │
│                         │  "Can't see this!"                    │
│                         │                                       │
│                         ■                                       │
│                    Tag 20 (Blue Goal)                           │
│                                                                 │
│   Problem: Robot loses localization when turning!               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Benefits of Multiple Cameras

| Benefit | Description | Impact |
|---------|-------------|--------|
| **360° Coverage** | See tags in all directions | Never lose localization |
| **Continuous Tracking** | Always have a tag in view while rotating | Smoother autonomous |
| **Redundancy** | Backup if one camera fails or is blocked | More reliable |
| **Improved Accuracy** | Average multiple detections | ±1" vs ±2-3" error |
| **Triangulation** | Multiple angles on same tag | Better 3D pose estimation |

---

## Single vs Multi-Camera Comparison

```
┌─────────────────────────────────────────────────────────────────┐
│              CAPABILITY COMPARISON                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   CAPABILITY                    │ 1 CAM │ 2 CAM │ 3+ CAM       │
│   ──────────────────────────────┼───────┼───────┼──────────    │
│   Basic tag detection           │   ✅   │   ✅   │   ✅         │
│   ftcPose (camera-relative)     │   ✅   │   ✅   │   ✅         │
│   robotPose (field position)    │   ✅   │   ✅   │   ✅         │
│   ──────────────────────────────┼───────┼───────┼──────────    │
│   360° field of view            │   ❌   │   ✅*  │   ✅         │
│   Continuous localization       │   ❌   │   ✅   │   ✅         │
│   Redundancy / fault tolerance  │   ❌   │   ✅   │   ✅         │
│   Multi-tag averaging           │   ❌   │   ✅   │   ✅         │
│   Better accuracy (averaged)    │   ❌   │   ✅   │   ✅         │
│   ──────────────────────────────┼───────┼───────┼──────────    │
│   Triangulation (same tag)      │   ❌   │   ✅*  │   ✅         │
│   Full surround awareness       │   ❌   │   ❌   │   ✅         │
│   ──────────────────────────────┼───────┼───────┼──────────    │
│   CPU usage                     │  Low  │  Med  │  High        │
│   Code complexity               │  Low  │  Med  │  High        │
│   Hardware cost                 │   $   │  $$   │  $$$         │
│                                                                 │
│   * = depends on camera placement                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### What You Get vs What You Must Implement

| Data/Capability | SDK Provides | You Must Implement |
|-----------------|--------------|-------------------|
| Separate detections per camera | ✅ | — |
| `ftcPose` per detection | ✅ | — |
| `robotPose` per detection | ✅ | — |
| Camera switching | ✅ | — |
| Simultaneous processing | ✅ | — |
| **Fusion of multiple robotPose** | ❌ | ✅ Your code |
| **Weighted averaging** | ❌ | ✅ Your code |
| **Kalman filtering** | ❌ | ✅ Your code |
| **Triangulation** | ❌ | ✅ Your code |

---

## SDK Multi-Camera Modes

The FTC SDK supports **two different approaches** for multiple cameras:

### Mode 1: Switchable Cameras (One Active at a Time)

```
┌─────────────────────────────────────────────────────────────────┐
│                 SWITCHABLE CAMERA MODE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌──────────┐         ┌──────────────────┐                     │
│   │ Webcam 1 │─────┐   │                  │                     │
│   └──────────┘     ├──►│  VisionPortal    │──► Detections       │
│   ┌──────────┐     │   │  (1 processor)   │                     │
│   │ Webcam 2 │─────┘   │                  │                     │
│   └──────────┘         └──────────────────┘                     │
│        ▲                        │                               │
│        │                        │                               │
│        └────── Switch command ──┘                               │
│                                                                 │
│   Only ONE camera active at any moment                          │
│   Switch takes ~200ms                                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**When to use:** Lower CPU usage, simpler code, when you don't need simultaneous coverage

```java
// Switchable camera setup
WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

CameraName switchableCamera = ClassFactory.getInstance()
    .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();

VisionPortal visionPortal = new VisionPortal.Builder()
    .setCamera(switchableCamera)
    .addProcessor(aprilTag)
    .build();

// Switch cameras on demand
visionPortal.setActiveCamera(webcam1);  // Use front camera
// ... later ...
visionPortal.setActiveCamera(webcam2);  // Switch to back camera
```

| Pros | Cons |
|------|------|
| Lower CPU usage | Only ONE camera active |
| Single processor to manage | ~200ms switching latency |
| Simpler detection handling | Must manually decide when to switch |

---

### Mode 2: Multi-Portal (Simultaneous Processing)

```
┌─────────────────────────────────────────────────────────────────┐
│                  MULTI-PORTAL MODE                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌──────────┐     ┌──────────────────┐                         │
│   │ Webcam 1 │────►│  VisionPortal 1  │──► Front detections     │
│   └──────────┘     │  (processor 1)   │                         │
│                    └──────────────────┘                         │
│                                                 ├──► Fusion     │
│   ┌──────────┐     ┌──────────────────┐         │    Algorithm  │
│   │ Webcam 2 │────►│  VisionPortal 2  │──► Back detections      │
│   └──────────┘     │  (processor 2)   │                         │
│                    └──────────────────┘                         │
│                                                                 │
│   BOTH cameras active simultaneously!                           │
│   True 360° coverage possible                                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**When to use:** Need continuous localization, 360° coverage, maximum reliability

```java
// Multi-portal setup (simultaneous cameras)
int[] viewIds = VisionPortal.makeMultiPortalView(2,
    VisionPortal.MultiPortalLayout.VERTICAL);

// Separate processor for EACH camera (required!)
AprilTagProcessor processor1 = new AprilTagProcessor.Builder()
    .setCameraPose(frontCameraPosition, frontCameraOrientation)
    .build();

AprilTagProcessor processor2 = new AprilTagProcessor.Builder()
    .setCameraPose(backCameraPosition, backCameraOrientation)
    .build();

// Separate portal for each camera
VisionPortal portal1 = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .setLiveViewContainerId(viewIds[0])
    .addProcessor(processor1)
    .build();

VisionPortal portal2 = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
    .setLiveViewContainerId(viewIds[1])
    .addProcessor(processor2)
    .build();

// Get detections from BOTH cameras
List<AprilTagDetection> frontDetections = processor1.getDetections();
List<AprilTagDetection> backDetections = processor2.getDetections();
```

| Pros | Cons |
|------|------|
| **TRUE simultaneous** processing | Higher CPU usage |
| No switching latency | Two processors to manage |
| 360° coverage possible | More complex code |
| Can fuse data from both | Need separate camera poses |

---

## Camera Placement Strategies

### Strategy 1: Front + Back (180° Coverage)

```
┌─────────────────────────────────────────────────────────────────┐
│                 FRONT + BACK CONFIGURATION                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                    Front Camera                                 │
│                   ╱ · · · · · ╲                                 │
│                  ╱             ╲                                │
│                 ╱   FRONT FOV   ╲                               │
│                ╱     (~60°)      ╲                              │
│               ╱                   ╲                              │
│   ───────────╱─────────────────────╲───────────                 │
│              │    ┌─────────┐      │                            │
│              │    │         │      │                            │
│   BLIND      │    │   🤖    │      │      BLIND                 │
│   (~60°)     │    │  Robot  │      │      (~60°)                │
│              │    │         │      │                            │
│              │    └─────────┘      │                            │
│   ───────────╲─────────────────────╱───────────                 │
│               ╲                   ╱                              │
│                ╲    BACK FOV    ╱                               │
│                 ╲    (~60°)    ╱                                │
│                  ╲           ╱                                  │
│                   ╲ · · · · ╱                                   │
│                    Back Camera                                  │
│                                                                 │
│   Coverage: ~240° (with overlap)                                │
│   Blind spots: Left and right sides                             │
│   Best for: Goal-focused navigation (DECODE)                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Best for DECODE:** Both goals are visible — Tag 24 (Red) in front OR back, Tag 20 (Blue) on the other side.

---

### Strategy 2: Left + Right (Side Coverage)

```
┌─────────────────────────────────────────────────────────────────┐
│                 LEFT + RIGHT CONFIGURATION                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                       BLIND                                     │
│                      (~60°)                                     │
│                         │                                       │
│   ───────────────────────────────────────────────               │
│                    ┌─────────┐                                  │
│      Left Cam     │         │     Right Cam                    │
│   ╱ · · · · · ╲   │         │   ╱ · · · · · ╲                  │
│  ╱             ╲  │   🤖    │  ╱             ╲                 │
│ ╱   LEFT FOV    ╲ │  Robot  │ ╱   RIGHT FOV   ╲                │
│╱     (~60°)      ╲│         │╱     (~60°)      ╲               │
│                   └─────────┘                                   │
│   ───────────────────────────────────────────────               │
│                         │                                       │
│                       BLIND                                     │
│                      (~60°)                                     │
│                                                                 │
│   Coverage: ~240° (sides)                                       │
│   Blind spots: Front and back                                   │
│   Best for: Side-approach navigation, wall following            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

### Strategy 3: Three Cameras (Near 360°)

```
┌─────────────────────────────────────────────────────────────────┐
│                 THREE CAMERA CONFIGURATION                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                    Front Camera                                 │
│                   ╱ · · · · · ╲                                 │
│                  ╱             ╲                                │
│                 ╱               ╲                               │
│                ╱                 ╲                              │
│   ╲           ╱                   ╲           ╱                 │
│    ╲         ╱   ┌─────────────┐   ╲         ╱                  │
│     ╲       ╱    │             │    ╲       ╱                   │
│      ╲     ╱     │     🤖      │     ╲     ╱                    │
│  Left ╲   ╱      │    Robot    │      ╲   ╱ Right               │
│  Cam   ╲ ╱       │             │       ╲ ╱  Cam                 │
│   · · · ╳        └─────────────┘        ╳ · · ·                 │
│        ╱ ╲                             ╱ ╲                      │
│       ╱   ╲                           ╱   ╲                     │
│      ╱     ╲         SMALL           ╱     ╲                    │
│     ╱       ╲      BLIND SPOT       ╱       ╲                   │
│    ╱         ╲       (~30°)        ╱         ╲                  │
│                                                                 │
│   Coverage: ~330°                                               │
│   Blind spot: Small rear area                                   │
│   Best for: Maximum coverage, complex autonomous                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

### Strategy 4: Angled Front Cameras (Wide Forward View)

```
┌─────────────────────────────────────────────────────────────────┐
│              ANGLED FORWARD CONFIGURATION                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│              Left-Front        Right-Front                      │
│                 Cam               Cam                           │
│                  ╲ · · · · · · · ╱                              │
│                   ╲             ╱                               │
│                    ╲    ∩∩    ╱                                 │
│                     ╲  WIDE ╱                                   │
│                      ╲ FOV ╱                                    │
│                       ╲   ╱                                     │
│                        ╲ ╱                                      │
│                    ┌────╳────┐                                  │
│                    │   🤖    │                                  │
│                    │  Robot  │                                  │
│                    └─────────┘                                  │
│                                                                 │
│   Coverage: ~150° forward (with stereo overlap)                 │
│   Blind spots: Sides and back                                   │
│   Best for: Stereo depth, forward-focused tasks                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Bonus:** Overlapping views enable **stereo depth estimation** for objects without AprilTags!

---

## Configuring Multiple Camera Poses

**CRITICAL:** Each camera needs its own position and orientation configuration for `robotPose` to work correctly.

### Measuring Camera Positions

```
┌─────────────────────────────────────────────────────────────────┐
│              CAMERA POSITION MEASUREMENT                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   TOP VIEW (measuring X and Y):                                 │
│   ─────────────────────────────                                 │
│                                                                 │
│           FRONT (+X direction)                                  │
│                  ▲                                              │
│                  │                                              │
│         ┌───────┴───────┐                                       │
│         │    Cam 1      │  Cam 1: x = +8", y = 0"               │
│         │    (front)    │                                       │
│         │       ●       │                                       │
│   ◄─────┤       │       ├─────►                                 │
│   +Y    │   ────●────   │    -Y                                 │
│  (left) │  Robot Center │  (right)                              │
│         │       │       │                                       │
│         │       ●       │                                       │
│         │    (back)     │  Cam 2: x = -8", y = 0"               │
│         │    Cam 2      │                                       │
│         └───────────────┘                                       │
│                  │                                              │
│                  ▼                                              │
│           BACK (-X direction)                                   │
│                                                                 │
│   SIDE VIEW (measuring Z):                                      │
│   ─────────────────────────                                     │
│                                                                 │
│         Cam ●────┐                                              │
│              │   │  z = 12" (camera height)                     │
│              │   │                                              │
│         ─────┴───┴─────  Ground                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Camera Orientation Reference

```
┌─────────────────────────────────────────────────────────────────┐
│              CAMERA ORIENTATION VALUES                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Camera Direction     │  Yaw  │ Pitch │ Roll │                 │
│   ─────────────────────┼───────┼───────┼──────┤                 │
│   Forward (default)    │   0°  │ -90°  │  0°  │                 │
│   Backward             │ 180°  │ -90°  │  0°  │                 │
│   Left                 │  90°  │ -90°  │  0°  │                 │
│   Right                │ -90°  │ -90°  │  0°  │                 │
│   Forward, tilted down │   0°  │ -105° │  0°  │  (15° down)     │
│   ─────────────────────┴───────┴───────┴──────┘                 │
│                                                                 │
│   Note: Pitch = -90° means camera is HORIZONTAL                 │
│         Pitch = 0° means camera points STRAIGHT UP              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Code: Configuring Two Cameras

```java
// ═══════════════════════════════════════════════════════════════
// FRONT CAMERA CONFIGURATION
// ═══════════════════════════════════════════════════════════════
// Camera is 8" forward from robot center, 12" high, facing forward
Position frontCamPosition = new Position(DistanceUnit.INCH,
    8.0,    // x: 8" forward
    0.0,    // y: centered (no left/right offset)
    12.0,   // z: 12" above ground
    0);

YawPitchRollAngles frontCamOrientation = new YawPitchRollAngles(
    AngleUnit.DEGREES,
    0,      // yaw: 0° = facing forward
    -90,    // pitch: -90° = horizontal
    0,      // roll: 0° = not tilted
    0);

AprilTagProcessor frontProcessor = new AprilTagProcessor.Builder()
    .setCameraPose(frontCamPosition, frontCamOrientation)
    .build();

// ═══════════════════════════════════════════════════════════════
// BACK CAMERA CONFIGURATION
// ═══════════════════════════════════════════════════════════════
// Camera is 8" backward from robot center, 10" high, facing backward
Position backCamPosition = new Position(DistanceUnit.INCH,
    -8.0,   // x: 8" backward (negative)
    0.0,    // y: centered
    10.0,   // z: 10" above ground
    0);

YawPitchRollAngles backCamOrientation = new YawPitchRollAngles(
    AngleUnit.DEGREES,
    180,    // yaw: 180° = facing backward
    -90,    // pitch: -90° = horizontal
    0,      // roll: 0° = not tilted
    0);

AprilTagProcessor backProcessor = new AprilTagProcessor.Builder()
    .setCameraPose(backCamPosition, backCamOrientation)
    .build();
```

---

## Fusion Algorithms

The SDK gives you separate detections from each camera. **You must implement fusion** to combine them into a single, more accurate robot pose.

### Algorithm 1: Simple Averaging

```java
/**
 * Simple averaging of all robotPose detections.
 * Best for: Quick implementation, roughly equal camera quality
 */
public Pose2d averageRobotPose(List<AprilTagDetection> allDetections) {
    double sumX = 0, sumY = 0, sumHeading = 0;
    int count = 0;

    for (AprilTagDetection detection : allDetections) {
        if (detection.robotPose != null && DecodeField.isGoalTag(detection.id)) {
            sumX += detection.robotPose.getPosition().x;
            sumY += detection.robotPose.getPosition().y;
            sumHeading += detection.robotPose.getOrientation()
                .getYaw(AngleUnit.RADIANS);
            count++;
        }
    }

    if (count == 0) return null;

    return new Pose2d(
        sumX / count,
        sumY / count,
        sumHeading / count
    );
}
```

---

### Algorithm 2: Distance-Weighted Averaging

```java
/**
 * Weight detections by inverse distance (closer = more trusted).
 * Best for: Variable detection distances, improved accuracy
 */
public Pose2d weightedAverageByDistance(List<AprilTagDetection> allDetections) {
    double sumX = 0, sumY = 0, sumHeading = 0;
    double totalWeight = 0;

    for (AprilTagDetection detection : allDetections) {
        if (detection.robotPose != null && detection.ftcPose != null
                && DecodeField.isGoalTag(detection.id)) {

            // Weight = 1 / distance (closer detections weighted more)
            double distance = detection.ftcPose.range;
            double weight = 1.0 / Math.max(distance, 1.0);  // Avoid div by zero

            sumX += detection.robotPose.getPosition().x * weight;
            sumY += detection.robotPose.getPosition().y * weight;
            sumHeading += detection.robotPose.getOrientation()
                .getYaw(AngleUnit.RADIANS) * weight;
            totalWeight += weight;
        }
    }

    if (totalWeight == 0) return null;

    return new Pose2d(
        sumX / totalWeight,
        sumY / totalWeight,
        sumHeading / totalWeight
    );
}
```

---

### Algorithm 3: Confidence-Weighted Averaging

```java
/**
 * Weight by detection confidence (decisionMargin).
 * Best for: Filtering out low-quality detections
 */
public Pose2d weightedAverageByConfidence(List<AprilTagDetection> allDetections) {
    double sumX = 0, sumY = 0, sumHeading = 0;
    double totalWeight = 0;

    for (AprilTagDetection detection : allDetections) {
        if (detection.robotPose != null && DecodeField.isGoalTag(detection.id)) {

            // Use decisionMargin as confidence weight
            double weight = detection.decisionMargin;

            // Optional: also factor in distance
            if (detection.ftcPose != null) {
                double distanceFactor = 1.0 / Math.max(detection.ftcPose.range, 12.0);
                weight *= distanceFactor;
            }

            sumX += detection.robotPose.getPosition().x * weight;
            sumY += detection.robotPose.getPosition().y * weight;
            sumHeading += detection.robotPose.getOrientation()
                .getYaw(AngleUnit.RADIANS) * weight;
            totalWeight += weight;
        }
    }

    if (totalWeight == 0) return null;

    return new Pose2d(
        sumX / totalWeight,
        sumY / totalWeight,
        sumHeading / totalWeight
    );
}
```

---

### Algorithm 4: Best Single Detection

```java
/**
 * Use only the "best" detection (closest + highest confidence).
 * Best for: Simplicity, when one detection is clearly better
 */
public AprilTagDetection getBestDetection(List<AprilTagDetection> allDetections) {
    AprilTagDetection best = null;
    double bestScore = Double.MAX_VALUE;

    for (AprilTagDetection detection : allDetections) {
        if (detection.robotPose != null && detection.ftcPose != null
                && DecodeField.isGoalTag(detection.id)) {

            // Score = distance / confidence (lower is better)
            double score = detection.ftcPose.range /
                Math.max(detection.decisionMargin, 1.0);

            if (score < bestScore) {
                bestScore = score;
                best = detection;
            }
        }
    }

    return best;
}
```

---

## Code Examples

### Complete Multi-Portal OpMode

```java
@TeleOp(name = "Multi-Camera AprilTag", group = "Vision")
public class MultiCameraAprilTag extends LinearOpMode {

    // Vision components
    private VisionPortal frontPortal, backPortal;
    private AprilTagProcessor frontProcessor, backProcessor;

    // Camera configurations
    private static final Position FRONT_CAM_POS = new Position(
        DistanceUnit.INCH, 8, 0, 12, 0);
    private static final YawPitchRollAngles FRONT_CAM_ORIENT =
        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private static final Position BACK_CAM_POS = new Position(
        DistanceUnit.INCH, -8, 0, 10, 0);
    private static final YawPitchRollAngles BACK_CAM_ORIENT =
        new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    @Override
    public void runOpMode() {
        initVision();

        telemetry.addLine("Multi-Camera AprilTag Ready");
        telemetry.addLine("Front + Back cameras active");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get detections from BOTH cameras
            List<AprilTagDetection> frontDetections = frontProcessor.getDetections();
            List<AprilTagDetection> backDetections = backProcessor.getDetections();

            // Combine all detections
            List<AprilTagDetection> allDetections = new ArrayList<>();
            allDetections.addAll(frontDetections);
            allDetections.addAll(backDetections);

            // Display per-camera info
            telemetry.addData("Front Camera Tags", frontDetections.size());
            telemetry.addData("Back Camera Tags", backDetections.size());
            telemetry.addData("Total Tags", allDetections.size());

            // Fuse into single robot pose
            Pose2d fusedPose = weightedAverageByDistance(allDetections);

            if (fusedPose != null) {
                telemetry.addLine("\n=== FUSED ROBOT POSE ===");
                telemetry.addData("X", "%.1f inches", fusedPose.getX());
                telemetry.addData("Y", "%.1f inches", fusedPose.getY());
                telemetry.addData("Heading", "%.1f°",
                    Math.toDegrees(fusedPose.getHeading()));
            } else {
                telemetry.addLine("\nNo valid detections for localization");
            }

            // Show individual detections
            for (AprilTagDetection d : allDetections) {
                if (d.metadata != null) {
                    telemetry.addLine(String.format("\nTag %d (%s): %.1f inches",
                        d.id, d.metadata.name, d.ftcPose.range));
                }
            }

            telemetry.update();
            sleep(20);
        }

        // Clean up
        frontPortal.close();
        backPortal.close();
    }

    private void initVision() {
        // Create split-screen view
        int[] viewIds = VisionPortal.makeMultiPortalView(2,
            VisionPortal.MultiPortalLayout.VERTICAL);

        // Front camera processor
        frontProcessor = new AprilTagProcessor.Builder()
            .setCameraPose(FRONT_CAM_POS, FRONT_CAM_ORIENT)
            .setDrawAxes(true)
            .build();

        // Back camera processor
        backProcessor = new AprilTagProcessor.Builder()
            .setCameraPose(BACK_CAM_POS, BACK_CAM_ORIENT)
            .setDrawAxes(true)
            .build();

        // Front portal
        frontPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setLiveViewContainerId(viewIds[0])
            .addProcessor(frontProcessor)
            .build();

        // Back portal
        backPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
            .setLiveViewContainerId(viewIds[1])
            .addProcessor(backProcessor)
            .build();
    }

    private Pose2d weightedAverageByDistance(List<AprilTagDetection> detections) {
        double sumX = 0, sumY = 0, sumHeading = 0;
        double totalWeight = 0;

        for (AprilTagDetection d : detections) {
            if (d.robotPose != null && d.ftcPose != null
                    && DecodeField.isGoalTag(d.id)) {
                double weight = 1.0 / Math.max(d.ftcPose.range, 1.0);

                sumX += d.robotPose.getPosition().x * weight;
                sumY += d.robotPose.getPosition().y * weight;
                sumHeading += d.robotPose.getOrientation()
                    .getYaw(AngleUnit.RADIANS) * weight;
                totalWeight += weight;
            }
        }

        if (totalWeight == 0) return null;

        return new Pose2d(sumX / totalWeight, sumY / totalWeight,
            sumHeading / totalWeight);
    }
}
```

---

## Practical Considerations

### CPU Usage

| Configuration | Approximate CPU Impact |
|---------------|------------------------|
| 1 camera, 640×480 | ~15-20% |
| 2 cameras (switchable) | ~15-20% (same as 1) |
| 2 cameras (simultaneous) | ~30-40% |
| 3 cameras (simultaneous) | ~45-60% |

**Tip:** Use lower resolution (640×480) for distant tags, higher (1280×720) only if needed.

### USB Bandwidth

```
┌─────────────────────────────────────────────────────────────────┐
│                    USB BANDWIDTH LIMITS                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Control Hub has LIMITED USB bandwidth!                        │
│                                                                 │
│   • 2 cameras at 640×480 @ 30fps → Usually OK                   │
│   • 2 cameras at 1280×720 @ 30fps → May cause issues            │
│   • 3+ cameras → Need USB hub or lower resolution               │
│                                                                 │
│   SYMPTOMS of bandwidth issues:                                 │
│   • Cameras disconnect randomly                                 │
│   • Frame rate drops significantly                              │
│   • "Camera not streaming" errors                               │
│                                                                 │
│   SOLUTIONS:                                                    │
│   • Use 640×480 resolution                                      │
│   • Lower frame rate if possible                                │
│   • Use powered USB hub                                         │
│   • Stagger camera initialization                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Hardware Mounting Tips

| Consideration | Recommendation |
|---------------|----------------|
| **Vibration** | Mount cameras with rubber dampeners |
| **Cable management** | Secure cables to prevent disconnection |
| **Field of view** | Ensure no robot parts block camera view |
| **Height** | Higher mounting = sees farther, but may miss close tags |
| **Protection** | Consider acrylic shields to prevent damage |

---

## Troubleshooting

### Problem: Only one camera works

**Possible causes:**
1. USB bandwidth exceeded
2. Same camera name in config
3. Portal initialization order

**Fix:**
```java
// Ensure different camera names
WebcamName cam1 = hardwareMap.get(WebcamName.class, "Webcam 1");  // Must match config
WebcamName cam2 = hardwareMap.get(WebcamName.class, "Webcam 2");  // Different name!

// Initialize portals with small delay
portal1 = buildPortal1();
sleep(500);  // Brief delay
portal2 = buildPortal2();
```

---

### Problem: robotPose values don't match between cameras

**Cause:** Camera poses configured incorrectly

**Fix:** Verify measurements:
1. Measure camera positions from robot CENTER
2. Double-check yaw angles (0° = forward, 180° = backward)
3. Test each camera individually first

---

### Problem: High CPU usage / lag

**Fix:**
```java
// Lower resolution
builder.setCameraResolution(new Size(640, 480));

// Increase decimation (faster but shorter range)
processor.setDecimation(3);  // Default, good balance

// Disable live view when not needed
portal.stopLiveView();
```

---

### Problem: Fused position jumps around

**Fix:** Add filtering:
```java
// Exponential moving average
private Pose2d smoothedPose = null;
private static final double SMOOTHING = 0.3;  // 0-1, lower = smoother

public Pose2d getSmoothedPose(Pose2d newPose) {
    if (smoothedPose == null) {
        smoothedPose = newPose;
    } else {
        smoothedPose = new Pose2d(
            smoothedPose.getX() * (1-SMOOTHING) + newPose.getX() * SMOOTHING,
            smoothedPose.getY() * (1-SMOOTHING) + newPose.getY() * SMOOTHING,
            smoothedPose.getHeading() * (1-SMOOTHING) + newPose.getHeading() * SMOOTHING
        );
    }
    return smoothedPose;
}
```

---

## Quick Reference

### Multi-Camera Mode Selection

| Need | Use |
|------|-----|
| Lower CPU, simple code | Switchable cameras |
| Continuous localization | Multi-portal |
| 360° coverage | Multi-portal + proper placement |
| Maximum accuracy | Multi-portal + fusion algorithm |

### Camera Orientation Quick Reference

| Camera Facing | Yaw | Pitch | Roll |
|---------------|-----|-------|------|
| Forward | 0° | -90° | 0° |
| Backward | 180° | -90° | 0° |
| Left | 90° | -90° | 0° |
| Right | -90° | -90° | 0° |

### Fusion Algorithm Selection

| Situation | Recommended Algorithm |
|-----------|----------------------|
| Quick implementation | Simple average |
| Variable distances | Distance-weighted |
| Noisy detections | Confidence-weighted |
| Want simplicity | Best single detection |
| Maximum accuracy | Kalman filter (advanced) |

---

## Learning Path

```
┌─────────────────────────────────────────────────────────────────┐
│              MULTI-CAMERA LEARNING PROGRESSION                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   LEVEL 1: Single Camera Mastery (Prerequisite)                 │
│   ──────────────────────────────────────────────                │
│   □ Complete AprilTag-Vision-Guide.md exercises                 │
│   □ Understand ftcPose and robotPose                            │
│   □ Configure camera pose correctly                             │
│                                                                 │
│   LEVEL 2: Switchable Cameras                                   │
│   ─────────────────────────────                                 │
│   □ Set up two cameras in hardware config                       │
│   □ Implement ConceptAprilTagSwitchableCameras                  │
│   □ Practice switching between cameras                          │
│                                                                 │
│   LEVEL 3: Simultaneous Multi-Portal                            │
│   ──────────────────────────────────                            │
│   □ Set up ConceptAprilTagMultiPortal                           │
│   □ Configure separate camera poses                             │
│   □ Verify both cameras detect tags independently               │
│                                                                 │
│   LEVEL 4: Basic Fusion                                         │
│   ─────────────────────                                         │
│   □ Implement simple averaging                                  │
│   □ Compare fused pose to individual poses                      │
│   □ Test while robot rotates 360°                               │
│                                                                 │
│   LEVEL 5: Advanced Fusion                                      │
│   ────────────────────────                                      │
│   □ Implement distance-weighted averaging                       │
│   □ Add confidence-based filtering                              │
│   □ Combine with odometry for sensor fusion                     │
│                                                                 │
│   LEVEL 6: Competition Ready                                    │
│   ──────────────────────────                                    │
│   □ Optimize for CPU usage                                      │
│   □ Add failure handling (camera disconnect)                    │
│   □ Test extensively in match conditions                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Resources

| Resource | Link |
|----------|------|
| **FTC Multi-Portal Guide** | [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/apriltag/vision_portal/vision_multiportal/vision-multiportal.html) |
| **VisionPortal Overview** | [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/apriltag/vision_portal/visionportal_overview/visionportal-overview.html) |
| **FTC Community: Two Cameras** | [ftc-community.firstinspires.org](https://ftc-community.firstinspires.org/t/using-two-cameras-simultaneously/1056) |

---

## Related Documentation

- [AprilTag-Vision-Guide.md](AprilTag-Vision-Guide.md) — Single camera AprilTag basics
- [L1-AutoAlign-Guide.md](L1-AutoAlign-Guide.md) — Using vision for goal alignment
- [AutoMode-Guide-Holonomic.md](AutoMode-Guide-Holonomic.md) — Autonomous navigation with vision
