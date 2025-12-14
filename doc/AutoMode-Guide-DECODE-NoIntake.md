# FTC DECODE (2025–2026) Autonomous Guide — Launcher-Only (No Intake)

**Last Updated:** 2025-12-14  
**FTC SDK Version:** 11.x (project-dependent)  
**Applies To:** Robots that only score **preloaded** artifacts/balls (no intake)

---

## Purpose

This guide documents a practical 30-second autonomous approach for a **launcher-only** FTC robot:

- Drive from the start area to a close, repeatable shooting position
- Use the **goal AprilTag** for alignment when available
- Shoot **3 preloaded** artifacts with consistent timing
- End positioned for TeleOp (optionally near the Manual Loading Zone if legal for your event/ruleset)

This is written to match typical implementations in this repo, especially:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleAutoOp.java`
- AprilTag examples like `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RobotAutoDriveToAprilTagTank.java`

---

## Key Concepts

### Alliance Selection (Red vs Blue)

Your auto should branch early based on alliance (e.g., during `init_loop()` with gamepad buttons):
- **Red vs Blue** usually mirrors turn direction and/or target heading.
- Keep distances the same where possible; only mirror angles/headings.

### Two AprilTags: “Motif/Obelisk” vs “Goal”

Your field has (at least) two relevant tag “roles”:
- **Obelisk / motif tag**: used to show pattern info; not intended for robot localization/approach.
- **Goal tag**: placed on/near the goal and is the correct one to align to for shooting.

**How to tell the difference in code (recommended):**
- Use the FTC tag library metadata and filter by name:
  - If `detection.metadata.name` contains `"Obelisk"`, treat it as motif/obelisk and ignore it for navigation.
  - Otherwise, treat it as a candidate “goal tag”.

Fallback (more robust once you know IDs at your event):
- Maintain a whitelist set of goal-tag IDs and only accept detections with IDs in that set.

---

## Recommended Auto Flow (No Intake)

### High-Level Timeline

1. **Init**: select alliance; spin up launcher (optional); confirm camera stream
2. **Leave start area**: drive forward a known distance to clear the starting corner/traffic
3. **Turn toward goal**: rotate to expected goal direction (mirrored for red/blue)
4. **Approach shooting position (close)**:
   - If a **goal AprilTag** is visible, use it to guide final approach/alignment
   - If not visible, dead-reckon to a pre-measured “close shot” waypoint
5. **Shoot 3 preloaded artifacts**: fixed cadence (example: ~2 seconds per shot including feed/settle)
6. **End for TeleOp**:
   - Park in a “safe” spot that doesn’t block your partner’s route
   - Optionally position near Manual Loading Zone **if it is not a protected/forbidden zone** (see below)

### Suggested State Machine

Use an enum-driven state machine (iterative `OpMode`) or sequential blocks (`LinearOpMode`):

- `SELECT_ALLIANCE` (usually in init)
- `DRIVE_CLEAR_START`
- `TURN_TOWARD_GOAL`
- `APPROACH_CLOSE_SHOT`
- `ALIGN_WITH_GOAL_TAG` (only if goal tag visible)
- `SHOOT_PRELOADS`
- `PARK_FOR_TELEOP`
- `COMPLETE`

---

## “Close as Possible” Shooting Guidance

Launcher-only robots commonly need to be **close** to score reliably. “Close” should mean:
- A **repeatable** robot pose (distance + heading), not “as close as you can get”
- A standoff distance that avoids:
  - bumping the goal
  - scraping field elements
  - being too close for the launcher’s trajectory (depends on your mechanism)

Practical approach:
- Pick a measured standoff distance (example values often start around 8–18 inches, but must be tuned).
- Tune one “close shot” pose for Red and mirror it for Blue.
- Use AprilTag alignment only for the last bit (fine corrections), not the whole drive.

---

## AprilTag Alignment (Goal Tag Only)

### What to Use from the Detection

Most FTC AprilTag auto-driving examples (including this repo’s examples) use:
- Range (how far you are from the tag)
- Bearing / yaw (how far left/right you are pointed from the tag centerline)

See: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RobotAutoDriveToAprilTagTank.java`

### Filtering Out the Obelisk Tag

Recommended rule:
- If `detection.metadata == null`: ignore (unknown tag)
- Else if `detection.metadata.name` contains `"Obelisk"`: ignore for navigation
- Else: accept as a goal tag candidate

### Fallback When No Goal Tag Is Visible

If you don’t see the goal tag:
- Continue dead-reckoning to your known “close shot” waypoint
- Take a short “settle” pause (100–300 ms) to reduce motion blur
- Retry AprilTag detection; if found, do quick alignment; if still not found, shoot from the waypoint

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

If you’re unsure at an event:
- Ask the Head Ref before matches: “Is it legal for the robot to enter the Manual Loading Zone during Autonomous?”

---

## Tuning Checklist (Do This on a Field)

- Measure and tune `DRIVE_CLEAR_START` distance so you don’t clip the wall/props.
- Tune the mirrored turn angle for Red vs Blue (start with a coarse angle, then refine).
- Tune your “close shot” waypoint:
  - Does it consistently see the goal tag?
  - Is the launcher accurate at that distance?
  - Are you stable (not rocking) while feeding shots?
- Tune shot cadence:
  - consistent flywheel spin-up (if applicable)
  - consistent feed timing between shots (example target: ~2 seconds/shot including settle)

---

## Match-Day Safety Checklist

- Verify alliance selection controls work in `init_loop()`.
- Verify the camera is oriented correctly (mirroring can swap left/right).
- Verify you are ignoring Obelisk tags for navigation.
- Verify you end auto in a TeleOp-friendly spot (no blocking, no illegal zone).

---

## Next Step (If You Want Code Changes)

If you want, I can refactor `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleAutoOp.java` to follow this sequence (drive → align → shoot → park) and add a goal-tag-only filter using `detection.metadata.name`.

