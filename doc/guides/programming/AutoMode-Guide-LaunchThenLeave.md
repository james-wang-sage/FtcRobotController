# FTC DECODE (2025–2026) Autonomous Guide — Launch Then Leave

**Last Updated:** 2025-01-15
**FTC SDK Version:** 11.x
**Applies To:** Robots that launch preloaded artifacts then exit the Launch Zone for LEAVE points
**Source Code:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pickle/PickleAutoLaunch.java`

---

## Purpose

This guide documents the **"Launch Then Leave"** autonomous strategy — a simple but effective 30-second autonomous that:

1. **Launches 3 preloaded balls** at the goal (while stationary in the Launch Zone)
2. **Waits** for a configurable delay (default 3 seconds)
3. **Moves diagonally** out of the Launch Zone using encoder-based movement
4. **Scores LEAVE** (3 Ranking Points contribution) by being fully outside the zone at AUTO end

This is an ideal autonomous for beginner teams because it:
- Requires **no vision processing** (no AprilTag alignment needed)
- Requires **no IMU** (simple encoder-based movement)
- Works for **both alliances** with automatic mirroring
- Contributes to **Movement RP** every match

---

## Strategy Overview

```
     FIELD VIEW (top-down)

         BLUE GOAL ◢              ◣ RED GOAL
              135° ↖              ↗ 45°

    ┌──────────────────────────────────────────┐
    │  BLUE                            RED     │
    │  LAUNCH                         LAUNCH   │
    │  ZONE                            ZONE    │
    │                                          │
    │   🤖 Blue                    Red 🤖      │
    │   faces 135°              faces 45°      │
    │        ↘                      ↙          │
    │         ╲   toward 180°     ╱            │
    │          ╲    (down)       ╱             │
    │           ↘               ↙              │
    │          (exit zone)   (exit zone)       │
    └──────────────────────────────────────────┘
                         ↓
                    180° (down toward alliance station)
```

### Timeline

```
0s              ~5s                  ~8s              ~10s     30s
│───────────────│────────────────────│────────────────│────────│
   LAUNCH 3      WAIT 3 SEC           MOVE DIAGONAL   DONE
   BALLS         (configurable)       (~900mm)        (LEAVE!)
```

---

## How It Works

### Phase 1: Launch Sequence (0–5 seconds)

1. **Spin up launcher** to target velocity (1100 ticks/sec)
2. **Wait** for launcher to reach minimum velocity (1000 ticks/sec)
3. **Feed balls** one at a time with 1-second intervals
4. **Stop launcher** after all balls are launched

```java
// Launcher configuration
private static final double LAUNCHER_TARGET_VELOCITY = 1100;  // ticks/second
private static final double LAUNCHER_MIN_VELOCITY = 1000;     // minimum before launching
private static final double FEED_TIME_SECONDS = 0.20;         // feeder run time per shot
private static final double INTERVAL_BETWEEN_SHOTS = 1.0;     // seconds between shots
private static final int TOTAL_BALLS = 3;                     // number of balls to launch
```

### Phase 2: Wait (5–8 seconds)

A configurable delay before moving. This allows:
- Balls to settle in the goal
- Alliance partner coordination
- Field elements to clear

```java
private static final double WAIT_BEFORE_DRIVE_SECONDS = 3.0;  // adjustable
```

### Phase 3: Exit Launch Zone (8–10 seconds)

The robot moves **diagonally** toward 180° (down on the field) to exit the Launch Zone.

#### Why Diagonal Movement?

The robot faces the **goal** (45° for Red, 135° for Blue) to launch. To exit toward 180° without rotating:

| Alliance | Robot Facing | Exit Direction | Relative Angle | Movement Type |
|----------|--------------|----------------|----------------|---------------|
| **RED** | 45° | 180° | 135° (back-right) | FR+BL pair, negative |
| **BLUE** | 135° | 180° | 45° (forward-right) | FL+BR pair, positive |

#### Mecanum Diagonal Movement

Mecanum wheels can move diagonally using only **one motor pair**:

```
MECANUM DIAGONAL PATTERNS

Back-Right (RED):           Forward-Right (BLUE):
  FL: OFF                     FL: FORWARD (+)
  FR: BACKWARD (-)            FR: OFF
  BL: BACKWARD (-)            BL: OFF
  BR: OFF                     BR: FORWARD (+)
```

### Phase 4: Stop and Display Results (10–30 seconds)

The robot stops all motors and displays:
- Alliance used
- Balls launched
- Distance moved (actual vs target)

---

## Encoder-Based Movement

### Why Encoders Instead of Time-Based?

| Aspect | Time-Based | Encoder-Based |
|--------|------------|---------------|
| **Battery dependency** | Distance varies with voltage | Consistent regardless of charge |
| **Repeatability** | Varies between runs | Same distance every time |
| **Feedback** | None | Real-time position tracking |
| **Debugging** | Hard to diagnose issues | Telemetry shows actual position |

### Encoder Constants

```java
// Motor encoder specifications (goBILDA 312 RPM motor)
private static final double COUNTS_PER_MOTOR_REV = 537.7;

// Wheel specifications (goBILDA 96mm mecanum wheels)
private static final double WHEEL_DIAMETER_MM = 96.0;

// Calculated: ticks per millimeter
private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);
// = 537.7 / (96 × 3.14159) = ~1.78 ticks/mm
```

### Distance Calculation

```java
// Distance to exit launch zone (~1.5 tiles = 36 inches = 914mm)
private static final double EXIT_DISTANCE_MM = 900.0;

// Diagonal correction factor (only 2 motors active = ~70% efficiency)
private static final double DIAGONAL_CORRECTION = 1.4;

// Final target in encoder ticks
int targetCounts = (int)(EXIT_DISTANCE_MM * COUNTS_PER_MM * DIAGONAL_CORRECTION);
// = 900 × 1.78 × 1.4 = ~2245 ticks
```

### RUN_TO_POSITION Mode

The code uses `RUN_TO_POSITION` mode for precise distance control:

```java
// 1. Reset encoders
motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// 2. Set target position
motor1.setTargetPosition(targetPosition);
motor2.setTargetPosition(targetPosition);

// 3. Switch to RUN_TO_POSITION
motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// 4. Apply power (must be positive for RUN_TO_POSITION)
motor1.setPower(DRIVE_SPEED);
motor2.setPower(DRIVE_SPEED);

// 5. Wait until target reached
while (motor1.isBusy() || motor2.isBusy()) {
    // Motors automatically stop when target is reached
}
```

---

## Alliance Selection

### During Init

Press gamepad buttons to select alliance **before** pressing Start:

| Button | Alliance | Diagonal Direction |
|--------|----------|-------------------|
| **A** | RED | Back-right (FR+BL) |
| **B** | BLUE | Forward-right (FL+BR) |

This matches the TeleOp alliance selection for consistency.

### Telemetry Display During Init

```
=== ALLIANCE SELECTION ===
Current Alliance: RED (A)
Press A: Select RED alliance
Press B: Select BLUE alliance

Strategy: Launch 3 balls → wait 3.0s → move 900mm
Diagonal: Back-RIGHT (FR+BL)
```

---

## Configuration Constants

All tunable parameters are at the top of the file:

### Launcher Settings

| Constant | Default | Description |
|----------|---------|-------------|
| `LAUNCHER_TARGET_VELOCITY` | 1100 | Launcher spin-up speed (ticks/sec) |
| `LAUNCHER_MIN_VELOCITY` | 1000 | Minimum speed before feeding |
| `FEED_TIME_SECONDS` | 0.20 | How long to run feeder per ball |
| `INTERVAL_BETWEEN_SHOTS` | 1.0 | Delay between shots (seconds) |
| `TOTAL_BALLS` | 3 | Number of balls to launch |

### Movement Settings

| Constant | Default | Description |
|----------|---------|-------------|
| `WAIT_BEFORE_DRIVE_SECONDS` | 3.0 | Delay after launching before moving |
| `DRIVE_SPEED` | 0.5 | Motor power (0.0 to 1.0) |
| `EXIT_DISTANCE_MM` | 900.0 | Distance to travel (~35 inches) |
| `DIAGONAL_CORRECTION` | 1.4 | Compensates for 2-motor diagonal |

### Encoder Settings

| Constant | Default | Description |
|----------|---------|-------------|
| `COUNTS_PER_MOTOR_REV` | 537.7 | goBILDA 312 RPM encoder CPR |
| `WHEEL_DIAMETER_MM` | 96.0 | goBILDA 96mm mecanum wheels |

---

## Tuning Guide

### If Robot Doesn't Move Far Enough

1. **Increase `EXIT_DISTANCE_MM`** (try 1000 or 1100)
2. **Increase `DIAGONAL_CORRECTION`** (try 1.5 or 1.6)
3. **Check battery charge** (encoders should compensate, but low battery affects motor response)

### If Robot Moves Too Far

1. **Decrease `EXIT_DISTANCE_MM`** (try 800 or 750)
2. **Decrease `DIAGONAL_CORRECTION`** (try 1.3 or 1.2)

### If Launcher Doesn't Reach Speed

1. **Increase spin-up wait time** (check telemetry for actual velocity)
2. **Check launcher motor wiring** and configuration
3. **Verify PIDF coefficients** in launcher initialization

### If Balls Don't Feed Properly

1. **Increase `FEED_TIME_SECONDS`** (try 0.25 or 0.30)
2. **Increase `INTERVAL_BETWEEN_SHOTS`** (try 1.2 or 1.5)
3. **Check feeder servo directions**

---

## Code Walkthrough

### File Structure

```
PickleAutoLaunch.java
├── Constants (lines 82-112)
│   ├── Launcher configuration
│   ├── Movement configuration
│   └── Encoder configuration
├── Hardware declarations (lines 114-128)
├── runOpMode() (line 130)
│   ├── INITIALIZATION (lines 132-186)
│   │   ├── Launcher setup
│   │   ├── Feeder setup
│   │   ├── Drive motor setup
│   │   └── Encoder reset
│   ├── ALLIANCE SELECTION (lines 192-217)
│   ├── LAUNCH SEQUENCE (lines 221-267)
│   ├── WAIT (lines 275-284)
│   ├── MOVE OUT OF ZONE (lines 287-364)
│   │   ├── Alliance-specific motor selection
│   │   ├── Encoder target calculation
│   │   └── RUN_TO_POSITION execution
│   └── CLEANUP (lines 366-384)
```

### Key Code Sections

#### Alliance-Specific Motor Selection

```java
DcMotor motor1, motor2;
int targetPosition;

if (alliance == Alliance.RED) {
    // RED: Back-right diagonal (FR+BL pair, negative direction)
    motor1 = frontRight;
    motor2 = backLeft;
    targetPosition = -targetCounts;  // Backward movement
} else {
    // BLUE: Forward-right diagonal (FL+BR pair, positive direction)
    motor1 = frontLeft;
    motor2 = backRight;
    targetPosition = targetCounts;   // Forward movement
}
```

#### Movement Loop with Timeout

```java
timer.reset();
double timeoutSeconds = 5.0;  // Safety timeout

while (opModeIsActive() &&
       (motor1.isBusy() || motor2.isBusy()) &&
       timer.seconds() < timeoutSeconds) {

    telemetry.addData("Motor1 Position", "%d / %d",
                      motor1.getCurrentPosition(), targetPosition);
    telemetry.addData("Motor2 Position", "%d / %d",
                      motor2.getCurrentPosition(), targetPosition);
    telemetry.update();
    sleep(50);
}
```

---

## Ranking Points Contribution

This autonomous contributes to **Movement RP** (threshold: 16+ points):

| Action | Points | This Auto |
|--------|--------|-----------|
| LEAVE (exit Launch Zone) | 3 pts | ✅ Yes |
| Partial BASE return | 5 pts | ❌ No (handled in TeleOp) |
| Full BASE return | 10 pts | ❌ No (handled in TeleOp) |

**To reach 16 points for Movement RP:**
- This auto: LEAVE = 3 pts
- Alliance partner LEAVE: +3 pts = 6 pts total
- Both robots full BASE return: +20 pts = **26 pts** ✅

See the [Ranking Points Guide](../competition/Ranking-Points-Guide.md) for complete RP strategy.

---

## Troubleshooting

### Robot Doesn't Move at All

1. **Check motor configuration names** match hardware map
2. **Verify motor directions** (right side should be REVERSE)
3. **Check encoder cables** are connected
4. **Look at telemetry** for error messages

### Robot Moves Wrong Direction

1. **Verify alliance selection** (A=RED, B=BLUE)
2. **Check motor direction settings** in code
3. **Test motors individually** to verify wiring

### Movement Stops Early

1. **Check timeout** (default 5 seconds should be enough)
2. **Verify encoder connections** (if encoder fails, isBusy() may return false)
3. **Increase `EXIT_DISTANCE_MM`** if robot reaches target but not outside zone

### Telemetry Shows Unexpected Values

1. **Check `COUNTS_PER_MM` calculation** matches your motor/wheel specs
2. **Verify `DIAGONAL_CORRECTION`** is appropriate for your robot
3. **Compare actual distance** traveled with telemetry reading

---

## Learning Path

### Level 1: Understand the Strategy
- [ ] Read this guide completely
- [ ] Watch the robot execute the auto (understand the sequence)
- [ ] Identify the Launch Zone on the field diagram

### Level 2: Test and Tune
- [ ] Run the auto with default settings
- [ ] Measure actual distance traveled
- [ ] Adjust `EXIT_DISTANCE_MM` if needed
- [ ] Test with both alliance settings

### Level 3: Understand the Code
- [ ] Read through `PickleAutoLaunch.java`
- [ ] Understand encoder math (COUNTS_PER_MM calculation)
- [ ] Understand mecanum diagonal movement patterns
- [ ] Trace alliance-specific code paths

### Level 4: Modify and Extend
- [ ] Add additional movements after LEAVE
- [ ] Integrate with AprilTag for more precise positioning
- [ ] Add pattern scoring on the ramp (advanced)

---

## Related Documentation

| Guide | Location | Description |
|-------|----------|-------------|
| Ranking Points Guide | `../competition/Ranking-Points-Guide.md` | Complete RP strategy |
| Mecanum Drive Guide | `../drivetrain/Mecanum-Drive-Guide.md` | Mecanum wheel mechanics |
| Motor Encoder Modes | `../drivetrain/Motor-Encoder-Modes-Guide.md` | RUN_TO_POSITION explained |
| TeleOp Guide | `TeleOp-Mode-Guide.md` | Driver control implementation |
| AutoMode DECODE (No Intake) | `AutoMode-Guide-DECODE-NoIntake.md` | Alternative auto with AprilTag |

---

## Resources

- [FTC Official Documentation](https://ftc-docs.firstinspires.org/)
- [DECODE Competition Manual](https://ftc-resources.firstinspires.org/ftc/game/cm-html/DECODE_Competition_Manual_TU18.htm)
- [goBILDA Motor Specs](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/)
- [Game Manual 0 - Encoders](https://gm0.org/en/latest/docs/software/concepts/encoders.html)

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2025-01-15 | 1.0 | Initial version with encoder-based movement and alliance mirroring |
