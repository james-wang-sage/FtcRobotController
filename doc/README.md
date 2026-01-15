# FTC Robot Controller Documentation

**Team Pickle - DECODE Season (2025-2026)**
**SDK Version:** 11.0

This folder contains all team documentation, guides, and reference materials for our FTC robot development.

---

## 📁 Folder Structure

```
doc/
├── guides/              # How-to guides and tutorials
│   ├── drivetrain/      # Drive system guides (Mecanum, motors, tuning)
│   ├── programming/     # OpMode patterns, TeleOp, Autonomous
│   └── vision/          # AprilTag and camera guides
├── images/              # Diagrams, photos, and visual assets
│   ├── field/           # Field layout and starting positions
│   ├── hardware/        # Robot hardware diagrams
│   └── software/        # Software architecture visuals
├── reference/           # Advanced patterns from other teams
├── team/                # Team-specific docs (booth board, notebook)
├── legal/               # License files (LEGO, audio assets)
└── media/               # Media assets (PR screenshots, etc.)
```

---

## 📚 Documentation Index

### Guides

#### Drivetrain
| Document | Description |
|----------|-------------|
| [Mecanum Drive Guide](guides/drivetrain/Mecanum-Drive-Guide.md) | Comprehensive guide to mecanum wheels and holonomic drive |
| [Omni Wheels Guide](guides/drivetrain/Omni-Wheels-Guide.md) | Understanding omni-directional wheel systems |
| [Mecanum Tuning Guide](guides/drivetrain/mecanum-tuning-guide.md) | Fine-tuning mecanum drive performance |
| [Mecanum Speed Guide](guides/drivetrain/MecanumSpeedGuide.md) | Optimizing speed and acceleration |
| [Motion System Overview](guides/drivetrain/Motion-System-Overview.md) | Complete motion control architecture |
| [Motor Encoder Modes](guides/drivetrain/Motor-Encoder-Modes-Guide.md) | Understanding encoder modes (RUN_USING_ENCODER, etc.) |

#### Programming
| Document | Description |
|----------|-------------|
| [OpMode Types Guide](guides/programming/OpMode-Types-Guide.md) | LinearOpMode vs OpMode comparison |
| [TeleOp Mode Guide](guides/programming/TeleOp-Mode-Guide.md) | Driver-controlled programming patterns |
| [AutoMode Guide (DECODE)](guides/programming/AutoMode-Guide-DECODE-NoIntake.md) | Autonomous for DECODE season |
| [AutoMode Guide (Holonomic)](guides/programming/AutoMode-Guide-Holonomic.md) | Autonomous with mecanum drive |
| [IMU TeleOp Guide](guides/programming/IMU-TeleOp-Guide.md) | Field-centric drive using IMU |
| [L1 AutoAlign Guide](guides/programming/L1-AutoAlign-Guide.md) | Automatic heading alignment |
| [BasicOpMode Comparison](guides/programming/BasicOpMode-vs-StarterBotTeleop-Comparison.md) | Understanding starter code differences |

#### Vision
| Document | Description |
|----------|-------------|
| [AprilTag Vision Guide](guides/vision/AprilTag-Vision-Guide.md) | AprilTag detection and localization |
| [Multi-Camera Guide](guides/vision/Multi-Camera-AprilTag-Guide.md) | Using multiple cameras for vision |

### Reference
| Document | Description |
|----------|-------------|
| [BunyipsFTC Patterns](reference/Advanced-Patterns-From-BunyipsFTC.md) | Advanced patterns from Australian teams |
| [Brighton FTC Patterns](reference/Advanced-Patterns-From-Brighton-FTC.md) | Patterns from Brighton FTC team |

### Team
| Document | Description |
|----------|-------------|
| [Booth Board Intro](team/Booth-Board-Intro.md) | Competition booth board content |
| [Engineering Notebook](team/Engineering-Notebook-PickleTeleOp.md) | PickleTeleOp development notes |

---

## 🎯 Quick Start

### New to FTC?
1. Start with [OpMode Types Guide](guides/programming/OpMode-Types-Guide.md)
2. Read [TeleOp Mode Guide](guides/programming/TeleOp-Mode-Guide.md)
3. Explore sample code in `FtcRobotController/src/main/java/.../samples/`

### Using Mecanum Drive?
1. [Mecanum Drive Guide](guides/drivetrain/Mecanum-Drive-Guide.md) - Understand the basics
2. [Motor Encoder Modes](guides/drivetrain/Motor-Encoder-Modes-Guide.md) - Configure motors correctly
3. [Mecanum Tuning Guide](guides/drivetrain/mecanum-tuning-guide.md) - Fine-tune performance

### Building Autonomous?
1. [AutoMode Guide (Holonomic)](guides/programming/AutoMode-Guide-Holonomic.md)
2. [AprilTag Vision Guide](guides/vision/AprilTag-Vision-Guide.md)
3. [L1 AutoAlign Guide](guides/programming/L1-AutoAlign-Guide.md)

---

## 🖼️ Visual Assets

### Field Diagrams
- `images/field/field.png` - 2D field layout
- `images/field/field.3d.png` - 3D field visualization
- `images/field/starting.locations.png` - Legal starting positions

### Hardware Diagrams
- `images/hardware/system.block.diagram.png` - System architecture
- `images/hardware/starter.kit.png` - Starter bot overview
- `images/hardware/mecanum.wheels.png` - Mecanum wheel arrangement
- `images/hardware/hardware.intro.png` - Hardware introduction visual

### Software Diagrams
- `images/software/software.intro.png` - Software architecture overview
- `images/software/software.intro.2.png` - Extended software diagram

---

## 📖 Learning Path

For team members new to FTC programming:

1. **Week 1-2: Basics**
   - Read [OpMode Types Guide](guides/programming/OpMode-Types-Guide.md)
   - Practice with BasicOpMode samples

2. **Week 3-4: TeleOp**
   - Study [TeleOp Mode Guide](guides/programming/TeleOp-Mode-Guide.md)
   - Learn [Mecanum Drive Guide](guides/drivetrain/Mecanum-Drive-Guide.md)

3. **Week 5-6: Autonomous**
   - Work through [AutoMode Guide](guides/programming/AutoMode-Guide-Holonomic.md)
   - Explore [AprilTag Vision](guides/vision/AprilTag-Vision-Guide.md)

4. **Week 7+: Advanced**
   - Study [BunyipsFTC Patterns](reference/Advanced-Patterns-From-BunyipsFTC.md)
   - Implement advanced features

---

## 🔗 External Resources

- **FTC Documentation:** https://ftc-docs.firstinspires.org/
- **Javadoc Reference:** https://javadoc.io/doc/org.firstinspires.ftc
- **FTC Community Forum:** https://ftc-community.firstinspires.org/
- **DECODE Competition Manual:** https://ftc-resources.firstinspires.org/ftc/game
- **GoBilda Starter Bot Guide:** https://www.gobilda.com/ftc-starter-bot-resource-guide-decode/

---

## 📝 Contributing

When adding new documentation:

1. Place guides in the appropriate subfolder under `guides/`
2. Add images to `images/` with descriptive names
3. Update this README with links to new documents
4. Follow existing naming conventions (Title-Case-With-Dashes.md)
5. Include a Table of Contents for longer documents
6. Add a "Learning Path" section with resource links when applicable

---

*Last Updated: January 2025*
