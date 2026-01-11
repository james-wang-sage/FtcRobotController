# FTC 2025-2026 DECODE Competition Booth Board

**Team Pickle - Hardware & Software Introduction**

This document contains the booth board content for the FTC DECODE competition season.

---

## Hardware

- **Foundation:** GoBilda FTC Starter Bot (DECODE)
- **Chassis Upgrade:** Strafer Kit with **Mecanum Wheels** — enables strafing & precise positioning
- **Drivetrain:** 4-motor AWD for full omnidirectional control
- **Launcher:** Velocity-controlled motor + dual servo feeders — **one-button 3-ball rapid-fire**
- **Custom:** Acrylic pusher panel for gate & ball control

### Hardware Details

| Component | Original (Starter Bot) | Upgraded |
|-----------|------------------------|----------|
| Chassis | GoBilda Starter Bot frame | Strafer Chassis Kit |
| Wheels | Standard drive wheels | 104mm GripForce Mecanum |
| Drive Motors | 2 motors (tank drive) | 4 motors (AWD) |
| Launcher | Stock ball launcher | Velocity-controlled with encoder feedback |
| Custom Parts | — | Acrylic front panel |

### Key Hardware Links

- [FTC Starter Bot Resource Guide](https://www.gobilda.com/ftc-starter-bot-resource-guide-decode/)
- [Strafer Chassis Kit (104mm GripForce Mecanum)](https://www.gobilda.com/strafer-chassis-kit-104mm-gripforce-mecanum-wheels/)

---

## Software

- **Foundation:** GoBilda Starter Code → heavily customized
- **Drive System:** Rewrote for **Mecanum wheels** — strafe + rotation + forward in any combo
- **Launcher:** Upgraded to **5-state machine** with timed 3-ball sequence & velocity recovery between shots
- **Auto Mode:** Simple & reliable — launch all 3 balls, park for TeleOp
- **Extras:** IMU auto-align, input deadband, strafe compensation

### Software Evolution

| Feature | Starter Code | Our Code |
|---------|--------------|----------|
| Drive System | 2-motor tank (arcade control) | 4-motor mecanum (omnidirectional) |
| Drive Algorithm | Simple arcade | Mecanum math + input shaping |
| Launcher States | 4 states (single shot) | 5 states (3-ball sequence) |
| Shot Timing | Manual button per ball | One-button rapid-fire |
| Velocity Control | Basic | Recovery between shots |
| Sensors | None | IMU for auto-align |

### Key Code Files

| File | Purpose |
|------|---------|
| `PickleTeleOp.java` | Driver-controlled mode with mecanum drive & 3-ball launcher |
| `PickleAutoOp.java` | Autonomous mode: launch 3 balls, then park |
| `StarterBotTeleop.java` | Original starter code (reference) |
| `StarterBotAuto.java` | Original starter auto (reference) |

### Software Highlights

#### 1. Mecanum Drive Algorithm

Rewrote the entire drive system to support omnidirectional movement:

```
Forward/Backward: All 4 wheels same direction
Strafing (side-to-side): Diagonal wheel pairs
Rotation: Left/right sides oppose each other
```

Formula:
```
Front Left  = forward + strafe + rotate
Front Right = forward - strafe - rotate
Back Left   = forward - strafe + rotate
Back Right  = forward + strafe - rotate
```

#### 2. 5-State Launcher State Machine

```
IDLE → SPIN_UP → LAUNCH → LAUNCHING → WAIT_BETWEEN
                              ↑            ↓
                              └────────────┘ (repeat for 3 balls)
```

- **Velocity recovery:** Waits for motor to reach target speed before each shot
- **Timed intervals:** 0.30s between balls for consistent trajectory
- **One-button operation:** Press once, fires all 3 balls automatically

#### 3. Auto Mode Strategy

Simple and reliable approach:
1. Launch 3 preloaded balls at the goal
2. Park in position for TeleOp handoff
3. No complex navigation (reduces failure points)

---

## Git History (Engineering Notebook Reference)

Key commits showing our development progress:

| Commit | Description |
|--------|-------------|
| `ac028b7` | Implement Mecanum drive system with improved speed settings |
| `58900f3` | Add multi-shot sequence functionality |
| `362614b` | Update feeder and launcher timings and implement velocity recovery |
| `867d290` | Update auto-align target headings and rotation speed constants |
| `8df74ff` | Update auto-align heading constants and mecanum drive |
| `68014db` | Add strafe compensation multiplier and telemetry updates |
| `b1f417d` | Refactor arcade drive control with deadband and normalization |

---

## Learning Resources

For teams looking to implement similar features:

- [FTC Official Documentation](https://ftc-docs.firstinspires.org/)
- [GoBilda FTC Resources](https://www.gobilda.com/ftc/)
- [Game Manual - DECODE](https://ftc-resources.firstinspires.org/ftc/game)
- See `/doc/Mecanum-Drive-Guide.md` for detailed mecanum wheel implementation
- See `/doc/Motor-Encoder-Modes-Guide.md` for velocity control concepts
