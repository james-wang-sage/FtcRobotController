# FTC DECODE (2025–2026) Autonomous Guide — Launcher-Only (No Intake)

**Last Updated:** 2025-12-14
**FTC SDK Version:** 11.x (project-dependent)
**Applies To:** Robots that only score **preloaded** artifacts/balls (no intake)
**Field Constants:** Uses official measurements from DECODE Competition Manual (Section 9: ARENA)

---

## Purpose

This guide documents a practical 30-second autonomous approach for a **launcher-only** FTC robot:

- Drive from the start area to a close, repeatable shooting position
- Use the **goal AprilTag** for alignment when available (graceful fallback if unavailable)
- Shoot **3 preloaded** artifacts with consistent timing
- End positioned for TeleOp (optionally near the Manual Loading Zone if legal for your event/ruleset)

This is written to match typical implementations in this repo, especially:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleAutoOp.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleHardwareNames.java`
- AprilTag examples like `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RobotAutoDriveToAprilTagTank.java`

---

## Key Concepts

### Alliance Selection (Red vs Blue)

Your auto should branch early based on alliance (e.g., during `init_loop()` with gamepad buttons):
- **Red vs Blue** usually mirrors turn direction and/or target heading.
- Keep distances the same where possible; only mirror angles/headings.

### AprilTag IDs (Official from Competition Manual)

The DECODE field uses specific AprilTag IDs defined in the official Competition Manual (Section 9: ARENA):

| Tag Location | AprilTag ID | Use for Navigation? |
|--------------|-------------|---------------------|
| **Red Alliance Goal** | 24 | ✅ YES - align to this for Red |
| **Blue Alliance Goal** | 20 | ✅ YES - align to this for Blue |
| **Obelisk Face 1** | 21 | ❌ NO - ignore for navigation |
| **Obelisk Face 2** | 22 | ❌ NO - ignore for navigation |
| **Obelisk Face 3** | 23 | ❌ NO - ignore for navigation |

**How to filter in code (using DecodeField helpers):**

```java
import org.firstinspires.ftc.teamcode.pickle.field.DecodeField;

// Get the goal tag ID for current alliance
int expectedGoalId = DecodeField.getGoalAprilTagId(alliance); // 24 for RED, 20 for BLUE

// Filter detections
for (AprilTagDetection detection : detections) {
    if (DecodeField.isObeliskTag(detection.id)) {
        continue; // Skip Obelisk tags (21, 22, 23)
    }
    if (DecodeField.isGoalTag(detection.id)) {
        // This is a goal tag (20 or 24) - use for alignment
        if (detection.id == expectedGoalId) {
            // This is OUR alliance's goal - prioritize it
        }
    }
}
```

**Why ID-based filtering is better than string matching:**
- Official tag IDs are stable constants from the Competition Manual
- String matching (`"Obelisk"`) could fail if metadata names change between SDK versions
- ID-based filtering is more explicit and self-documenting

### Hardware Name Consistency (recommended)

FTC hardware devices are looked up by **string name** from the Driver Station configuration. To avoid typos and drift
between Auto/TeleOp, keep all device names in one place:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleHardwareNames.java`

Current device names:
| Constant | Value | Description |
|----------|-------|-------------|
| `FRONT_LEFT_MOTOR` | `"front_left"` | Mecanum drive motor |
| `FRONT_RIGHT_MOTOR` | `"front_right"` | Mecanum drive motor |
| `BACK_LEFT_MOTOR` | `"back_left"` | Mecanum drive motor |
| `BACK_RIGHT_MOTOR` | `"back_right"` | Mecanum drive motor |
| `LAUNCHER_MOTOR` | `"launcher"` | High-speed launcher motor |
| `LEFT_FEEDER_SERVO` | `"left_feeder"` | Continuous rotation servo |
| `RIGHT_FEEDER_SERVO` | `"right_feeder"` | Continuous rotation servo |
| `WEBCAM_NAME` | `"Webcam 1"` | USB camera for AprilTag |
| `IMU_NAME` | `"imu"` | Built-in REV Hub IMU |

### Official Field Measurements (from Competition Manual)

The `DecodeField.java` class contains official measurements from the DECODE Competition Manual (Section 9: ARENA):

| Element | Dimensions | DecodeField Constant |
|---------|-----------|----------------------|
| **Field Size** | 144" × 144" (12' × 12') | `FieldConstants.FIELD_SIZE_INCHES` |
| **Tile Size** | 24" × 24" | `FieldConstants.TILE_SIZE_INCHES` |
| **Base Zone** | 18" × 18" ± 0.125" | `DecodeField.BASE_SIZE_MM` |
| **Loading Zone** | ~23" × 23" | `DecodeField.LOADING_ZONE_SIZE_MM` |
| **Gate Zone** | 2.75" × 10" | `DecodeField.GATE_WIDTH_MM`, `GATE_LENGTH_MM` |
| **Secret Tunnel** | 46.5" × 6.125" | `DecodeField.SECRET_TUNNEL_LENGTH_MM`, `WIDTH_MM` |
| **Goal Structure** | 27" × 27" × 54" tall | `DecodeField.GOAL_WIDTH_MM`, `DEPTH_MM`, `HEIGHT_MM` |
| **Goal Opening Height** | 38.75" from tile | `DecodeField.GOAL_OPENING_HEIGHT_MM` |

**Coordinate System Convention:**
- Origin (0, 0) is at field center
- +X points from Blue alliance toward Red alliance
- +Y points from audience side toward far side
- All internal units are millimeters

**Using field positions in code:**

```java
import org.firstinspires.ftc.teamcode.pickle.field.*;
import org.firstinspires.ftc.teamcode.pickle.geometry.*;

// Get alliance-specific positions
Translation2d goalCenter = DecodeField.getGoalCenter(alliance);
Rectangle2d baseRegion = DecodeField.getBaseRegion(alliance);
Pose2d startPose = DecodeField.getStartingPose(alliance, 1);

// Check if robot is in a zone
if (DecodeField.isInBase(currentPose, alliance)) {
    telemetry.addData("Parking", "SUCCESS!");
}

// Calculate heading to goal
double headingToGoal = DecodeField.headingToGoal(currentPosition, alliance);
```

---

## Graceful Degradation Philosophy

The autonomous is designed with **layered fallbacks** so it completes even when hardware is unreliable:

1. **IMU available** → Uses gyroscope for accurate heading control
2. **IMU fails** → Falls back to encoder-based rotation (arc length calculation)
3. **Camera available** → Uses AprilTag for final alignment
4. **Camera fails** → Falls back to dead-reckoning path
5. **Motor stalls** → Timeout prevents autonomous from hanging

**Key principle:** Vision and IMU are enhancements, not dependencies. The autonomous must score using dead-reckoning alone.

---

## Recommended Auto Flow (No Intake)

### High-Level Timeline

1. **Init**: select alliance; initialize IMU and camera (with try-catch); confirm telemetry shows status
2. **Leave start area**: drive forward a known distance to clear the starting corner/traffic
3. **Turn toward goal**: rotate to expected goal direction (mirrored for red/blue) — uses IMU if available
4. **Approach shooting position (close)**:
   - If a **goal AprilTag** is visible, use it to guide final approach/alignment
   - If not visible, dead-reckon to a pre-measured "close shot" waypoint
5. **Shoot 3 preloaded artifacts**: fixed cadence (example: ~2 seconds per shot including feed/settle)
6. **End for TeleOp**:
   - Park in a "safe" spot that doesn't block your partner's route
   - Optionally position near Manual Loading Zone **if it is not a protected/forbidden zone** (see below)

### Suggested State Machine

Use an enum-driven state machine (iterative `OpMode`) or sequential blocks (`LinearOpMode`):

- `START_DELAY` (optional; partner de-conflict)
- `DRIVE_CLEAR_START`
- `TURN_TOWARD_GOAL`
- `DRIVE_TO_CLOSE_SHOT`
- `ALIGN_WITH_GOAL_TAG` (time-bounded; ignores Obelisk tags; falls back if no goal tag visible)
- `REQUEST_SHOT` / `WAIT_FOR_SHOT` (repeat 3x)
- `PARK_FOR_TELEOP`
- `COMPLETE`

---

## Movement Functions

### Mecanum Drive Capabilities

The autonomous supports three movement types for mecanum drive:

| Function | Description | Use Case |
|----------|-------------|----------|
| `drive()` | Forward/backward movement | Main path navigation |
| `strafe()` | Lateral (sideways) movement | Fine-tuning position without rotating |
| `rotate()` | In-place rotation | Turning toward goal |

### IMU-Based Rotation (Recommended)

When `USE_IMU_FOR_ROTATION = true` and the IMU is available, rotation uses **closed-loop heading control**:

```java
// IMU rotation uses actual gyroscope heading
YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
double headingError = targetHeadingDeg - currentHeading;
// Apply proportional power to reduce error
```

**Why this is better:** Encoder-based rotation calculates arc length from track width, which assumes perfect traction. On competition carpet, wheel slip can cause 10-20% heading error. The IMU measures actual heading directly.

**IMU Orientation:** The code assumes the REV Hub is mounted with:
- Logo facing UP
- USB port facing FORWARD

If your hub is mounted differently, update the `initIMU()` method with your orientation.

### Encoder-Based Rotation (Fallback)

If the IMU fails or `USE_IMU_FOR_ROTATION = false`, rotation falls back to encoder-based arc calculation:

```java
double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
```

### Strafe Movement

Mecanum strafe uses the diagonal roller pattern:
- **Strafe RIGHT:** FL forward, FR backward, BL backward, BR forward
- **Strafe LEFT:** FL backward, FR forward, BL forward, BR backward

The `STRAFE_CORRECTION = 1.2` factor compensates for lateral movement being less efficient than forward movement due to roller geometry.

---

## Safety Features

### Motor Timeout Protection

Every `drive()`, `strafe()`, and `rotate()` call has a **maximum timeout**:

| Constant | Default Value | Purpose |
|----------|---------------|---------|
| `DRIVE_TIMEOUT_SECONDS` | 5.0 | Max time for drive/strafe operations |
| `ROTATE_TIMEOUT_SECONDS` | 4.0 | Max time for rotation operations |

If the robot hasn't reached its target position within this time, the operation completes anyway and the autonomous continues. This prevents hanging if a motor stalls or an encoder fails.

### All-Motor Position Checking

Position tolerance is verified on **all 4 motors**, not just one:

```java
boolean allMotorsAtTarget =
    Math.abs(targetPosition - frontLeft.getCurrentPosition()) <= toleranceTicks &&
    Math.abs(targetPosition - frontRight.getCurrentPosition()) <= toleranceTicks &&
    Math.abs(targetPosition - backLeft.getCurrentPosition()) <= toleranceTicks &&
    Math.abs(targetPosition - backRight.getCurrentPosition()) <= toleranceTicks;
```

This ensures the robot is actually at the target position, not just that one motor reached it.

### Hardware Initialization Safety

Both IMU and camera initialization are wrapped in try-catch blocks:

```java
private void initIMU() {
    try {
        imu = hardwareMap.get(IMU.class, PickleHardwareNames.IMU_NAME);
        // ... configuration ...
        imuAvailable = true;
    } catch (Exception e) {
        imu = null;
        imuAvailable = false;
        telemetry.addData("IMU", "Init failed - using encoder rotation");
    }
}
```

If initialization fails, the autonomous continues with fallback behavior.

---

## "Close as Possible" Shooting Guidance

Launcher-only robots commonly need to be **close** to score reliably. "Close" should mean:
- A **repeatable** robot pose (distance + heading), not "as close as you can get"
- A standoff distance that avoids:
  - bumping the goal
  - scraping field elements
  - being too close for the launcher's trajectory (depends on your mechanism)

Practical approach:
- Pick a measured standoff distance (example values often start around 8–18 inches, but must be tuned).
- Tune one "close shot" pose for Red and mirror it for Blue.
- Use AprilTag alignment only for the last bit (fine corrections), not the whole drive.

---

## Partner Collision Avoidance (with current hardware)

With encoders + AprilTags, you still can't reliably detect and avoid another robot. The safest approach is coordination
and deterministic paths:

- Agree pre-match who "owns" the goal approach lane in auto.
- Add an optional start delay so only one robot enters the goal area first.
- Keep a "safe auto" option that just clears the start area and parks (no goal approach).
- Approach the goal at reduced speed for the final segment.

---

## AprilTag Alignment (Goal Tag Only)

### What to Use from the Detection

Most FTC AprilTag auto-driving examples (including this repo's examples) use:
- Range (how far you are from the tag)
- Bearing / yaw (how far left/right you are pointed from the tag centerline)

See: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RobotAutoDriveToAprilTagTank.java`

### Filtering AprilTags Using DecodeField

The `getGoalDetection()` method in `PickleAutoOp.java` uses `DecodeField` helpers for robust filtering:

```java
private AprilTagDetection getGoalDetection() {
    int expectedTagId = DecodeField.getGoalAprilTagId(alliance);

    for (AprilTagDetection detection : detections) {
        // Skip unknown tags
        if (detection.metadata == null || detection.ftcPose == null) {
            continue;
        }

        // Skip Obelisk tags (IDs 21, 22, 23) - NOT for navigation
        if (DecodeField.isObeliskTag(detection.id)) {
            continue;
        }

        // Check if this is a goal tag (IDs 20 or 24)
        if (DecodeField.isGoalTag(detection.id)) {
            if (detection.id == expectedTagId) {
                // Prioritize our alliance's goal tag
            }
        }
    }
}
```

**Selection priority:**
1. First, look for the current alliance's goal tag (most reliable)
2. If not found, accept any goal tag (opponent's goal might be visible)
3. Always reject Obelisk tags using `DecodeField.isObeliskTag()`

### Practical Alignment Pattern (recommended)

For predictability, prefer small "nudges" over continuous closed-loop driving:

- If bearing error is large, do a small turn nudge (IMU-based or encoder-based rotate).
- Else if range error is large, do a small forward/back nudge (encoder-based drive).
- **NEW:** Consider using `strafe()` for lateral fine-tuning without changing heading.
- Stop after a short time budget and shoot even if the tag is lost.

If the robot turns the wrong direction during alignment (camera mounting/orientation), set `INVERT_TAG_TURN = true` in code.

### Fallback When No Goal Tag Is Visible

If you don't see the goal tag:
- Continue dead-reckoning to your known "close shot" waypoint
- The alignment state has a timeout (`GOAL_ALIGN_MAX_SECONDS = 1.5`)
- After timeout, proceed to shooting from the dead-reckoned position

---

## Manual Loading Zone vs Starting Zone (Can You Drive There in Auto?)

Based on the info we pulled so far:
- The **Manual Loading Zone** is primarily about **human actions** (humans loading/handling artifacts), which are typically **TeleOp-only**.
- We did **not** confirm a rule that explicitly bans the *robot* from entering the Manual Loading Zone during Autonomous.

What to do in practice (recommended):
1. Open the official DECODE manual + combined team updates for your event:
   - `https://ftc-resources.firstinspires.org/ftc/game/manual`
   - `https://ftc-resources.firstinspires.org/ftc/game/tu-combined`
2. Search for: `"Loading Zone"`, `"Protected"`, `"Autonomous"`, `"opponent"`, `"incursion"`.
3. If the Manual Loading Zone is defined as a **protected zone** (or opponent-protected zone), follow that restriction.
4. If it is **not protected**, parking near/inside it at end of auto is usually acceptable, but still avoid blocking your partner.

If you're unsure at an event:
- Ask the Head Ref before matches: "Is it legal for the robot to enter the Manual Loading Zone during Autonomous?"

---

## Tunable Constants Reference

### Path Distances and Angles

| Constant | Default | Description |
|----------|---------|-------------|
| `START_DELAY_SECONDS` | 0.0 | Optional delay before starting (partner coordination) |
| `DRIVE_CLEAR_START_IN` | 18.0 | Distance to drive away from starting corner |
| `TURN_TOWARD_GOAL_DEG` | 45.0 | Rotation angle toward goal (mirrored for blue) |
| `DRIVE_TO_CLOSE_SHOT_IN` | 36.0 | Distance to drive to shooting position |
| `PARK_BACKUP_IN` | -12.0 | Distance to backup after shooting (negative = backward) |

### AprilTag Alignment

| Constant | Default | Description |
|----------|---------|-------------|
| `GOAL_ALIGN_MAX_SECONDS` | 1.5 | Max time to spend on AprilTag alignment |
| `GOAL_ALIGN_DESIRED_RANGE_IN` | 10.0 | Target distance from goal tag |
| `GOAL_ALIGN_RANGE_TOL_IN` | 2.0 | Acceptable range error |
| `GOAL_ALIGN_BEARING_TOL_DEG` | 2.0 | Acceptable bearing error |
| `GOAL_ALIGN_TURN_STEP_DEG` | 5.0 | Size of turn nudges |
| `GOAL_ALIGN_DRIVE_STEP_IN` | 3.0 | Size of drive nudges |
| `INVERT_TAG_TURN` | false | Flip if robot turns wrong way during alignment |

### IMU and Rotation

| Constant | Default | Description |
|----------|---------|-------------|
| `USE_IMU_FOR_ROTATION` | true | Enable IMU-based heading control |
| `IMU_HEADING_TOLERANCE_DEG` | 2.0 | How close to target heading before "done" |

### Safety Timeouts

| Constant | Default | Description |
|----------|---------|-------------|
| `DRIVE_TIMEOUT_SECONDS` | 5.0 | Max time for any drive/strafe operation |
| `ROTATE_TIMEOUT_SECONDS` | 4.0 | Max time for any rotation operation |

### Drive Speeds

| Constant | Default | Description |
|----------|---------|-------------|
| `DRIVE_SPEED` | 0.5 | Speed for main driving |
| `ROTATE_SPEED` | 0.2 | Speed for rotation |

---

## Tuning Checklist (Do This on a Field)

### IMU Calibration
- [ ] Let robot sit still for 2-3 seconds after init before pressing Start
- [ ] Verify IMU telemetry shows "Active" (not "Fallback to encoders")
- [ ] If IMU shows wrong heading, check hub orientation in `initIMU()`

### Path Tuning
- [ ] Measure and tune `DRIVE_CLEAR_START_IN` so you don't clip the wall/props
- [ ] Tune `TURN_TOWARD_GOAL_DEG` for Red vs Blue (should be mirrored)
- [ ] Tune `DRIVE_TO_CLOSE_SHOT_IN` to reach a consistent shooting position

### Strafe Calibration (if using strafe)
- [ ] Command a 24" strafe, measure actual distance
- [ ] Adjust `STRAFE_CORRECTION` factor (typical range: 1.1-1.4)

### Shooting Position
- [ ] Does it consistently see the goal tag? (check telemetry)
- [ ] Is the launcher accurate at that distance?
- [ ] Are you stable (not rocking) while feeding shots?

### Shot Cadence
- [ ] Consistent flywheel spin-up (check `LAUNCHER_TARGET_VELOCITY`)
- [ ] Consistent feed timing between shots (`TIME_BETWEEN_SHOTS = 2`)

### Timeout Values
- [ ] If timeouts trigger during normal operation (see "WARNING" telemetry), increase them
- [ ] If robot hangs too long on failed moves, decrease them

---

## Match-Day Safety Checklist

- [ ] Verify alliance selection controls work in `init_loop()` (Press B for Red, X for Blue)
- [ ] Verify telemetry shows:
  - `IMU: Active` or `IMU: Fallback to encoders`
  - `Camera: Initialized successfully` or `Camera: Init failed - using dead reckoning`
  - `Goal AprilTag ID: 24` (for Red) or `Goal AprilTag ID: 20` (for Blue)
- [ ] Verify the camera is oriented correctly (mirroring can swap left/right)
- [ ] Verify you are ignoring Obelisk tags for navigation:
  - Telemetry shows `[OBELISK-IGNORE]` label on tags 21, 22, 23
  - Telemetry shows `[GOAL]` or `[OUR GOAL]` on tags 20 or 24
- [ ] Verify you end auto in a TeleOp-friendly spot (no blocking, no illegal zone)
- [ ] Test auto with camera unplugged to verify dead-reckoning fallback works

---

## Troubleshooting

### Robot Turns Wrong Direction During AprilTag Alignment
- Set `INVERT_TAG_TURN = true` in PickleAutoOp.java

### Rotation Overshoots or Undershoots
- If using IMU: Adjust `IMU_HEADING_TOLERANCE_DEG`
- If using encoders: Verify `TRACK_WIDTH_MM` matches your robot

### Strafe Drifts Forward/Backward
- This is normal for mecanum — ensure all motors are properly reversed
- Check that motor directions in `init()` match your wiring

### Autonomous Hangs (Doesn't Progress)
- Check telemetry for "WARNING: Drive timeout" or "WARNING: Rotate timeout"
- If timeouts aren't triggering, they may be set too high
- Verify encoders are plugged in (check motor positions in telemetry)

### AprilTag Never Detected
- Verify camera is connected and shows in configuration
- Check `GOAL_ALIGN_MAX_SECONDS` — if too short, you may skip alignment before camera focuses
- Ensure lighting is adequate for camera exposure

### Wrong AprilTag Being Used
- Verify telemetry shows correct expected goal tag ID (24 for Red, 20 for Blue)
- Check that Obelisk tags (21, 22, 23) show `[OBELISK-IGNORE]` in telemetry
- If aligning to opponent's goal, check alliance selection before match

---

## Learning Path

For teams new to FTC autonomous programming:

1. **Start Simple**: Run the autonomous with `USE_IMU_FOR_ROTATION = false` and no AprilTag alignment
2. **Add IMU**: Enable IMU and verify rotation accuracy improves
3. **Add Vision**: Enable AprilTag alignment and tune the nudge parameters
4. **Add Strafe**: Use strafe for fine lateral adjustments

### Resources

- [FTC Official Documentation](https://ftc-docs.firstinspires.org/)
- [FTC Javadoc Reference](https://javadoc.io/doc/org.firstinspires.ftc)
- [FTC Community Forum](https://ftc-community.firstinspires.org/)
- [Game Manual & Team Updates](https://ftc-resources.firstinspires.org/ftc/game)

---

## Files in This Implementation

| File | Purpose |
|------|---------|
| `pickle/PickleAutoOp.java` | Main autonomous OpMode with state machine |
| `pickle/PickleHardwareNames.java` | Centralized hardware device names |
| `pickle/PickleTeleOp.java` | Corresponding TeleOp (uses same hardware names) |
| `pickle/field/DecodeField.java` | DECODE-specific field positions (goals, ramps, bases, loading zones) |
| `pickle/field/FieldConstants.java` | Base field dimensions (144" × 144") and coordinate system |
| `pickle/field/Alliance.java` | RED/BLUE enum with alliance-aware helpers |
| `pickle/geometry/Pose2d.java` | 2D pose (x, y, heading) for robot position |
| `pickle/geometry/Translation2d.java` | 2D translation (x, y) for field points |
| `pickle/geometry/Rectangle2d.java` | Axis-aligned bounding box for field regions |
| `doc/AutoMode-Guide-DECODE-NoIntake.md` | This documentation |
