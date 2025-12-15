# L1 Auto-Align Button Guide

> **Quick-rotate to the optimal launching angle** with a single button press.

## Table of Contents

1. [What Does L1 Do?](#what-does-l1-do)
2. [What L1 Does and Does NOT Do](#important-what-l1-does-and-does-not-do)
3. [Understanding the IMU Limitation](#understanding-the-imu-limitation)
4. [Driver Responsibilities](#driver-responsibilities)
5. [Target Headings Explained](#target-headings-explained)
6. [How to Use L1 — Step by Step](#how-to-use-l1--step-by-step)
7. [State Machine Diagram](#state-machine-diagram)
8. [Telemetry Reference](#telemetry-reference)
9. [Troubleshooting](#troubleshooting)
10. [Quick Reference Card](#quick-reference-card)
11. [Summary](#summary)
12. [Adding Position Awareness (Future Enhancement)](#adding-position-awareness-future-enhancement)

---

## What Does L1 Do?

**Hold L1 (Left Trigger)** → Robot automatically rotates to face **perpendicular to the goal zone border** for optimal ball launching.

```
┌─────────────────────────────────────────────────────────────────┐
│                     L1 AUTO-ALIGN IN ACTION                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   BEFORE L1:                       AFTER L1:                    │
│   Robot facing random direction    Robot facing goal opening    │
│                                                                 │
│          RED GOAL                        RED GOAL               │
│            ╲                               ╲                    │
│             ╲                               ╲                   │
│              ╲                               ╲  ←── 135° target │
│               ╲                               ╲                 │
│                                                                 │
│         ┌───┐                          ┌───┐                    │
│         │ → │  Facing 60°              │ ↖ │  Facing 135°       │
│         └───┘                          └───┘                    │
│                                                                 │
│   Driver holds L1...                   ALIGNED! Ready to fire   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Important: What L1 Does and Does NOT Do

### ✅ L1 DOES:

| Feature | Description |
|---------|-------------|
| **Rotate robot** | Automatically turns to face the correct angle |
| **Use IMU heading** | Reads current orientation from the IMU sensor |
| **Target alliance goal** | 135° for RED, 45° for BLUE |
| **Allow driving during align** | You can still move forward/backward and strafe |

### ❌ L1 Does NOT:

| Limitation | Why |
|------------|-----|
| **Know robot's position** | IMU only provides heading, not X/Y location |
| **Drive to optimal spot** | Driver must position the robot manually |
| **Know distance to goal** | No position tracking in TeleOp mode |
| **Auto-aim at goal center** | Rotates to fixed angle, not toward a target |

---

## Understanding the IMU Limitation

```
┌─────────────────────────────────────────────────────────────────┐
│                 WHAT THE IMU ACTUALLY KNOWS                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   The IMU is like a COMPASS — it knows DIRECTION, not LOCATION  │
│                                                                 │
│   ✅ IMU KNOWS:                    ❌ IMU DOES NOT KNOW:        │
│   ─────────────                    ──────────────────────       │
│                                                                 │
│   "Robot is facing 87°"            "Robot is at position (X,Y)" │
│                                                                 │
│   "Robot rotated 45° left"         "Robot is 3 feet from goal"  │
│                                                                 │
│   "Robot is tilted 5°"             "Robot is near the wall"     │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ANALOGY:                                                      │
│   ─────────                                                     │
│                                                                 │
│   🧭 Compass: "You're facing North"     ← This is what IMU does │
│                                                                 │
│   📍 GPS: "You're at 123 Main Street"   ← IMU CANNOT do this    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Driver Responsibilities

Since L1 only handles **rotation**, the driver must handle **positioning**:

```
┌─────────────────────────────────────────────────────────────────┐
│                    DRIVER vs ROBOT RESPONSIBILITIES             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   👤 DRIVER MUST:                  🤖 ROBOT (L1) WILL:          │
│   ───────────────                  ─────────────────            │
│                                                                 │
│   • Drive to launching position    • Rotate to target heading   │
│   • Judge distance to goal         • Maintain alignment         │
│   • Avoid obstacles                • Show heading in telemetry  │
│   • Choose when to align           • Stop rotating when aligned │
│   • Press RB to launch             • Allow driving during align │
│                                                                 │
│                    TEAMWORK! 🤝                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Target Headings Explained

The goal zones have **45° angled borders**. To launch balls straight into the opening, the robot must face **perpendicular** to this angle.

```
┌─────────────────────────────────────────────────────────────────┐
│                  DECODE FIELD GOAL GEOMETRY                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                    TOP OF FIELD (+Y)                            │
│         ┌─────────────────────────────────────┐                 │
│         │                                     │                 │
│         │   BLUE GOAL           RED GOAL      │                 │
│         │      ╲                   ╱          │                 │
│         │       ╲ 135°       45° ╱           │                  │
│         │        ╲             ╱              │                 │
│         │         ╲           ╱               │                 │
│         │          ╲         ╱                │                 │
│         │                                     │                 │
│   (-X)  │           FIELD CENTER              │  (+X)           │
│   BLUE  │              (72,72)                │  RED            │
│   SIDE  │                                     │  SIDE           │
│         │                                     │                 │
│         │                                     │                 │
│         │                                     │                 │
│         └─────────────────────────────────────┘                 │
│                   AUDIENCE (-Y)                                 │
│                                                                 │
│   Field Coordinate System:                                      │
│     0° = facing right (+X, toward Red side)                     │
│    90° = facing up (+Y, toward far wall)                        │
│   180° = facing left (-X, toward Blue side)                     │
│   270° = facing down (-Y, toward audience)                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Target Heading by Alliance

| Alliance | Goal Location | Border Angle | **Perpendicular Heading** | Direction |
|----------|---------------|--------------|---------------------------|-----------|
| **RED** | Top-right corner | 45° | **135°** | Northwest ↖ |
| **BLUE** | Top-left corner | 135° | **45°** | Northeast ↗ |

```
   RED Alliance Target (135°):        BLUE Alliance Target (45°):

         ↖                                   ↗
          ╲                                 ╱
           ╲ 135°                     45° ╱
            ╲                           ╱
             ●                         ●
           Robot                     Robot
```

---

## How to Use L1 — Step by Step

### Pre-Match Setup

```
┌─────────────────────────────────────────────────────────────────┐
│  STEP 1: SELECT ALLIANCE (during init_loop)                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Press X → BLUE alliance (target: 45°)                         │
│   Press B → RED alliance (target: 135°)                         │
│                                                                 │
│   ⚠️  IMPORTANT: Select BEFORE pressing START!                  │
│                                                                 │
│   Telemetry will show:                                          │
│   ┌──────────────────────────────┐                              │
│   │ Current Alliance: RED        │                              │
│   │ Target Heading: 135.0°       │                              │
│   │ Current IMU Heading: 87.3°   │                              │
│   └──────────────────────────────┘                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### During Match

```
┌─────────────────────────────────────────────────────────────────┐
│  STEP 2: DRIVE TO LAUNCHING POSITION                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Use joysticks to drive robot near the goal zone               │
│                                                                 │
│   💡 Tips:                                                      │
│   • Get within effective launching range                        │
│   • Don't worry about exact angle yet                           │
│   • Clear line of sight to goal opening                         │
│                                                                 │
│            RED GOAL                                             │
│              ╲                                                  │
│               ╲                                                 │
│                ╲                                                │
│                 ╲                                               │
│                                                                 │
│           ┌───┐                                                 │
│           │ ? │  ← Robot in position (angle doesn't matter yet) │
│           └───┘                                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  STEP 3: HOLD L1 TO AUTO-ALIGN                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Hold Left Trigger (L1) → Robot starts rotating                │
│                                                                 │
│   Watch telemetry:                                              │
│   ┌────────────────────────────────────────┐                    │
│   │ Align State: ALIGNING                  │                    │
│   │ Heading: 87.3° → 135.0° (err: 47.7°)   │                    │
│   │          ↑        ↑           ↑        │                    │
│   │       current  target      error       │                    │
│   └────────────────────────────────────────┘                    │
│                                                                 │
│   Robot rotates automatically toward 135° (for RED)             │
│                                                                 │
│   ⚠️  You can still drive forward/backward and strafe!          │
│       Only rotation is controlled by L1                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  STEP 4: WAIT FOR "ALIGNED" STATE                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   When error is within ±3°, state changes to ALIGNED:           │
│                                                                 │
│   ┌────────────────────────────────────────┐                    │
│   │ Align State: ALIGNED        ✓ READY!   │                    │
│   │ Heading: 134.2° → 135.0° (err: 0.8°)   │                    │
│   └────────────────────────────────────────┘                    │
│                                                                 │
│           ┌───┐                                                 │
│           │ ↖ │  ← Robot now facing 135° (perpendicular to goal)│
│           └───┘                                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  STEP 5: LAUNCH!                                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   While still holding L1:                                       │
│                                                                 │
│   Press RB (Right Bumper) → Launch sequence starts              │
│                                                                 │
│   ┌────────────────────────────────────────┐                    │
│   │ Launch State: SPIN_UP → LAUNCH         │                    │
│   │ Launcher Speed: 1125                   │                    │
│   └────────────────────────────────────────┘                    │
│                                                                 │
│   🎯 Ball launches toward goal!                                 │
│                                                                 │
│   Release L1 when done → Return to manual control               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## State Machine Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                   L1 AUTO-ALIGN STATE MACHINE                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                        ┌──────────┐                             │
│                   ┌───▶│   IDLE   │◀───┐                        │
│                   │    │          │    │                        │
│                   │    └────┬─────┘    │                        │
│                   │         │          │                        │
│      L1 Released  │         │ L1 Pressed                        │
│                   │         ▼          │                        │
│                   │    ┌──────────┐    │                        │
│                   │    │ ALIGNING │    │ L1 Released            │
│                   │    │          │────┘                        │
│                   │    │ Robot is │                             │
│                   │    │ rotating │                             │
│                   │    └────┬─────┘                             │
│                   │         │                                   │
│                   │         │ Within ±3° of target              │
│                   │         ▼                                   │
│                   │    ┌──────────┐                             │
│                   └────│ ALIGNED  │                             │
│                        │          │                             │
│                        │ Ready to │                             │
│                        │ launch!  │                             │
│                        └──────────┘                             │
│                                                                 │
│   IDLE     = Normal manual control                              │
│   ALIGNING = Robot rotating toward target heading               │
│   ALIGNED  = Target reached, holding position                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Telemetry Reference

### During Alignment

```
┌────────────────────────────────────────────────────────────────┐
│                    TELEOP TELEMETRY                            │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  Alliance: RED                 ← Your selected alliance        │
│  Drive Mode: NORMAL (70%)      ← Speed mode                    │
│  Launch State: IDLE            ← Launcher status               │
│                                                                │
│  Align State: ALIGNING         ← L1 is being held              │
│  Heading: 87.3° → 135.0° (err: 47.7°)                          │
│           ───┬──   ───┬──   ────┬────                          │
│              │        │         │                              │
│         Current    Target    How much                          │
│         heading    heading   to rotate                         │
│                                                                │
│  Launch Status: READY          ← Can fire                      │
│  Launcher Speed: 0             ← Flywheel velocity             │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### Telemetry Values Explained

| Value | Meaning | What to Watch For |
|-------|---------|-------------------|
| **Align State** | Current alignment status | Wait for `ALIGNED` |
| **Heading (current)** | Where robot is facing now | Updates in real-time |
| **Heading (target)** | Where robot should face | 135° (RED) or 45° (BLUE) |
| **Heading (error)** | Difference from target | Should approach 0° |

---

## Troubleshooting

### Problem: Robot rotates the wrong direction (long way around)

**Example:** Robot at 170° rotating to 135° goes through 0° instead of directly

**Cause:** IMU orientation might be inverted

**Fix:** Check IMU USB direction in code matches physical mounting:
```java
RevHubOrientationOnRobot.UsbFacingDirection.FORWARD  // Try BACKWARD if wrong
```

---

### Problem: Robot won't reach target (keeps oscillating)

**Cause:** Tolerance too tight or P-gain issues

**Fix:** Increase tolerance in `PickleTeleOp.java`:
```java
final double ALIGN_TOLERANCE_DEG = 5.0;  // Was 3.0
```

---

### Problem: Robot overshoots target heading

**Cause:** Rotation too aggressive

**Fix:** Reduce speed and P-gain:
```java
final double ALIGN_ROTATION_SPEED = 0.25;  // Was 0.35
final double ALIGN_HEADING_KP = 0.010;     // Was 0.015
```

---

### Problem: L1 does nothing

**Possible causes:**
1. IMU not initialized (check telemetry for "IMU: NOT AVAILABLE")
2. Left trigger not pressed hard enough (threshold is 0.5)
3. Already in ALIGNED state

**Fix:** Check telemetry for IMU status and try pressing trigger firmly

---

### Problem: Wrong target heading for my alliance

**Cause:** Alliance not selected before START

**Fix:** During init phase, press:
- **X** for BLUE (45° target)
- **B** for RED (135° target)

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────┐
│              L1 AUTO-ALIGN QUICK REFERENCE                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  BEFORE MATCH:                                                  │
│    Press X = BLUE (target 45°)                                  │
│    Press B = RED (target 135°)                                  │
│                                                                 │
│  DURING MATCH:                                                  │
│    1. Drive to launching position (YOU control this)            │
│    2. Hold L1 (Left Trigger)                                    │
│    3. Robot auto-rotates to target heading                      │
│    4. Wait for "Align State: ALIGNED"                           │
│    5. Press RB to launch                                        │
│    6. Release L1 to return to manual control                    │
│                                                                 │
│  REMEMBER:                                                      │
│    ✅ L1 controls ROTATION only                                 │
│    ❌ L1 does NOT know robot POSITION                           │
│    👤 YOU must drive to the right spot                          │
│                                                                 │
│  TARGET HEADINGS:                                               │
│    RED  = 135° (facing ↖ northwest toward goal)                 │
│    BLUE = 45°  (facing ↗ northeast toward goal)                 │
│                                                                 │
│  TELEMETRY TO WATCH:                                            │
│    Align State: IDLE → ALIGNING → ALIGNED                       │
│    Heading: current° → target° (error°)                         │
│                                                                 │
│  IMU = COMPASS (knows direction, NOT location)                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Summary

| Question | Answer |
|----------|--------|
| What does L1 do? | Auto-rotates robot to face perpendicular to goal |
| Does it know position? | ❌ No — IMU only provides heading |
| Who positions the robot? | 👤 The driver |
| Target for RED? | 135° (northwest) |
| Target for BLUE? | 45° (northeast) |
| When is it aligned? | Error within ±3° |
| Can I drive during align? | ✅ Yes — forward, backward, strafe work |

---

## Adding Position Awareness (Future Enhancement)

Currently, L1 only knows **heading** (from IMU). To know **position**, we need additional sensors.

### How Autonomous Mode Knows Position

Your **PickleAutoHolonomic** (autonomous mode) DOES know full position using sensor fusion:

| Sensor | What It Provides |
|--------|------------------|
| **Wheel Encoders** | Track distance traveled → estimate X, Y position |
| **AprilTag Camera** | See tags on field → calculate absolute position |
| **IMU** | Heading for rotation accuracy |

```
┌─────────────────────────────────────────────────────────────────┐
│              SENSOR FUSION (Autonomous Mode)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Encoders ──────▶ ┌──────────────┐                             │
│   (X, Y estimate)  │              │                             │
│                    │   SENSOR     │ ──▶ Full Pose (X, Y, θ)     │
│   AprilTag ──────▶ │   FUSION     │     Knows WHERE + FACING    │
│   (absolute pos)   │              │                             │
│                    └──────────────┘                             │
│   IMU ───────────▶       ▲                                      │
│   (heading θ)            │                                      │
│                          │                                      │
│                  Combines all sensors                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Could We Add Position Awareness to TeleOp?

**Yes!** Here's what it would take:

| Option | Complexity | What It Enables |
|--------|------------|-----------------|
| **Add AprilTag vision** | Medium | Know position when goal tags visible |
| **Add encoder odometry** | Medium | Track position continuously |
| **Full sensor fusion** | High | Most accurate, like autonomous |

### Features Enabled by Position Tracking

If we added position tracking to TeleOp, L1 could be enhanced to:

```
┌─────────────────────────────────────────────────────────────────┐
│            POTENTIAL L1 ENHANCEMENTS WITH POSITION              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   CURRENT L1 (heading only):                                    │
│   ──────────────────────────                                    │
│   ✅ Rotate to target angle                                     │
│   ❌ Driver judges position                                     │
│                                                                 │
│   ENHANCED L1 (with position):                                  │
│   ────────────────────────────                                  │
│   ✅ Rotate to target angle                                     │
│   ✅ Show "Distance to Goal: 24 inches" in telemetry            │
│   ✅ Warn if too far: "⚠️ Move closer for optimal launch"       │
│   ✅ Warn if too close: "⚠️ Back up - too close!"               │
│   ✅ Auto-drive to optimal launch spot (hold L1 longer)         │
│   ✅ Field-centric driving (up = toward goal, always)           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Implementation Path

To add position awareness to TeleOp:

```
┌─────────────────────────────────────────────────────────────────┐
│                  IMPLEMENTATION ROADMAP                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   PHASE 1: Encoder Odometry (Medium effort)                     │
│   ──────────────────────────────────────────                    │
│   • Add MecanumOdometry to TeleOp                               │
│   • Track X, Y position from wheel encoders                     │
│   • Show position in telemetry                                  │
│   • ⚠️ Drifts over time without correction                      │
│                                                                 │
│   PHASE 2: AprilTag Vision (Medium effort)                      │
│   ────────────────────────────────────────                      │
│   • Initialize AprilTagLocalizer                                │
│   • Detect goal tags when visible                               │
│   • Calculate absolute robot position                           │
│   • Show distance to goal in telemetry                          │
│                                                                 │
│   PHASE 3: Full Sensor Fusion (Higher effort)                   │
│   ─────────────────────────────────────────                     │
│   • Combine encoders + AprilTag + IMU                           │
│   • Continuous position updates (encoders)                      │
│   • Periodic corrections (AprilTag)                             │
│   • Most accurate positioning                                   │
│                                                                 │
│   PHASE 4: Enhanced L1 (After position works)                   │
│   ──────────────────────────────────────────                    │
│   • L1 tap = rotate only (current behavior)                     │
│   • L1 hold 1 sec = rotate + show distance                      │
│   • L1 hold 2 sec = auto-drive to optimal spot                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Code Already Available

Good news! The position tracking code already exists in your codebase:

| Component | File | Can Reuse? |
|-----------|------|------------|
| `MecanumOdometry` | `pickle/odometry/MecanumOdometry.java` | ✅ Yes |
| `AprilTagLocalizer` | `pickle/vision/AprilTagLocalizer.java` | ✅ Yes |
| `MecanumDriveHelper` | `pickle/drive/MecanumDriveHelper.java` | ✅ Yes |
| `DecodeField` | `pickle/field/DecodeField.java` | ✅ Yes |
| `Pose2d` / `Translation2d` | `pickle/geometry/` | ✅ Yes |

These components are used by `PickleAutoHolonomic` and can be added to `PickleTeleOp` when ready.

### Quick Comparison: Current vs Enhanced

| Feature | Current L1 | Enhanced L1 (Future) |
|---------|------------|----------------------|
| Auto-rotate to goal | ✅ | ✅ |
| Know robot heading | ✅ (IMU) | ✅ (IMU) |
| Know robot position | ❌ | ✅ (Encoders + AprilTag) |
| Show distance to goal | ❌ | ✅ |
| Warn if out of range | ❌ | ✅ |
| Auto-drive to launch spot | ❌ | ✅ |
| Field-centric drive option | ❌ | ✅ |

---

## Related Documentation

- [TeleOp Mode Guide](TeleOp-Mode-Guide.md) — Complete TeleOp controls
- [IMU TeleOp Guide](IMU-TeleOp-Guide.md) — Understanding the IMU
- [Mecanum Drive Guide](Mecanum-Drive-Guide.md) — How mecanum wheels work
