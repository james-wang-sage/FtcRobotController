/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pickle;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pickle.config.PickleHardwareNames;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It uses a 4-motor MECANUM DRIVE system
 * for full omnidirectional robot mobility, one high-speed motor driving two "launcher wheels",
 * and two servos which feed that launcher.
 *
 * MECANUM WHEEL CONFIGURATION (4 Motors):
 * This robot uses goBILDA 104mm mecanum wheels with 4 motors (one per wheel).
 * This provides FULL omnidirectional movement:
 * - Forward/backward movement: Left stick Y-axis
 * - Strafing (side-to-side): Left stick X-axis
 * - Rotation: Right stick X-axis
 *
 * WHEEL ARRANGEMENT (viewed from above):
 *     FRONT
 *   FL     FR     (FL = Front Left, FR = Front Right)
 *     \   /       Arrows show roller direction
 *     /   \
 *   BL     BR     (BL = Back Left, BR = Back Right)
 *     BACK
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "PickleTeleOp", group = "StarterBot")
public class PickleTeleOp extends OpMode {
    /*
     * REQUIRED HARDWARE CONFIGURATION (configured in Driver Station app):
     * - "front_left"    : DcMotor (mecanum drive motor, front left)
     * - "front_right"   : DcMotor (mecanum drive motor, front right)
     * - "back_left"     : DcMotor (mecanum drive motor, back left)
     * - "back_right"    : DcMotor (mecanum drive motor, back right)
     * - "launcher"      : DcMotorEx (high-speed launcher motor with encoder)
     * - "left_feeder"   : CRServo (continuous rotation servo)
     * - "right_feeder"  : CRServo (continuous rotation servo)
     *
     * HARDWARE: goBILDA FTC Starter Kit 2025-2026 + 1 extra motor
     * https://www.gobilda.com/ftc-starter-kit-2025-2026-season/
     *
     * MOTOR CONFIGURATION (5× goBILDA 5203 Series Yellow Jacket, 312 RPM):
     * - 4× Drive motors: 312 RPM (19.2:1 ratio), 537.7 CPR at output shaft
     * - 1× Launcher motor: 312 RPM (19.2:1 ratio), 537.7 CPR at output shaft
     *
     * SERVO SPECIFICATIONS (goBILDA 2000 Series Dual Mode):
     * - Left/Right feeders: Continuous rotation mode
     *
     * CONTROL HUB PORT MAPPING (recommended):
     * - Port 0: front_left
     * - Port 1: front_right
     * - Port 2: back_left
     * - Port 3: back_right
     * - Expansion Hub Port 0: launcher
     */

    final double FEED_TIME_SECONDS = 0.35; //The feeder servos run this long when a shot is requested.
    final double SHOT_INTERVAL_SECONDS = 0.30; //Time between balls in a multi-shot sequence (for feeder reset)
    final double SPIN_UP_TIMEOUT_SECONDS = 1.0; //Max time to wait for velocity recovery before firing anyway
    final int BALLS_PER_LAUNCH = 3; //Number of balls to fire per button press
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * LAUNCHER VELOCITY CONFIGURATION
     *
     * These values control the launcher motor's speed using encoder feedback.
     *
     * HARDWARE: goBILDA 5203 Series Yellow Jacket Motor (312 RPM, 19.2:1)
     *
     * IMPORTANT: The FTC SDK measures velocity at the OUTPUT SHAFT (after gearbox),
     * NOT at the motor shaft.
     *
     * MOTOR SPECS:
     *   - Gear Ratio: 19.2:1
     *   - Output RPM: 312 RPM (no load @ 12V)
     *   - Encoder CPR: 537.7 counts per revolution at output shaft
     *   - Max Velocity: 312/60 × 537.7 = 2,796 ticks/sec
     *
     * CURRENT CONFIGURATION:
     *   - Target: 1,400 ticks/sec = 156 RPM output (50% of max)
     *   - Min: 1,300 ticks/sec = 145 RPM output (spin-up threshold)
     *
     * TUNING THESE VALUES:
     * 1. Start with a lower target velocity (e.g., 1000) for safety
     * 2. Gradually increase until launcher performs consistently
     * 3. Set MIN_VELOCITY about 50-100 ticks below TARGET for reliability
     * 4. Monitor telemetry "Launcher Speed" to see actual velocity
     */
    final double LAUNCHER_TARGET_VELOCITY = 1100;  // Target speed in encoder ticks/second
    final double LAUNCHER_MIN_VELOCITY = 1000;     // Minimum speed before allowing launch

    /*
     * AUTO-ALIGN HEADING CONSTANTS
     *
     * COORDINATE SYSTEMS:
     *
     * 1. FIELD COORDINATE SYSTEM (absolute, math convention):
     *    - Origin at (0,0) = bottom-left corner of field
     *    - 0° = right (+X direction)
     *    - 90° = up (+Y direction, toward far wall)
     *    - 180° = left (-X direction)
     *    - 270° = down (-Y direction)
     *    - Counter-clockwise = positive rotation
     *
     * 2. ROBOT/IMU COORDINATE SYSTEM (relative, reset at match start):
     *    - 0° = wherever robot faces at match start (typically 90° field = far wall)
     *    - Positive angles = counter-clockwise (turn left)
     *    - Negative angles = clockwise (turn right)
     *    - IMU heading = Field heading - 90° (when robot starts facing far wall)
     *
     * DECODE FIELD LAYOUT (viewed from above):
     *
     *       (0,144)                              (144,144)
     *          ┌──────────────────────────────────────┐
     *          │  BLUE GOAL ◢              ◣ RED GOAL │  ← FAR WALL (field 90°)
     *          │  Field: 135°              Field: 45° │
     *          │  IMU: +45°                IMU: -45°  │
     *          │                                      │
     *          │              90° (up/+Y)             │
     *          │                  ↑                   │
     *          │       180° ←─────┼─────→ 0°          │  ← Field compass
     *          │         (-X)     │      (+X)         │
     *          │                  ↓                   │
     *          │              270° (down/-Y)          │
     *          │                                      │
     *          │  [RED START]            [BLUE START] │  ← NEAR WALL (alliance stations)
     *          └──────────────────────────────────────┘
     *       (0,0)                                (144,0)
     *
     * ROBOT STARTING POSITION:
     *   - Robot starts facing the FAR WALL (field 90°, +Y direction)
     *   - IMU yaw is reset to 0° at match start
     *   - This makes: IMU reading = Field angle - 90°
     *
     * TARGET HEADINGS (IMU values):
     *
     *   RED Goal (top-right corner, field 45°):
     *     - Turn RIGHT (clockwise) 45° from start
     *     - IMU target = -45°
     *
     *   BLUE Goal (top-left corner, field 135°):
     *     - Turn LEFT (counter-clockwise) 45° from start
     *     - IMU target = +45°
     *
     * TUNING: If alignment is off, adjust these values during practice.
     * Use telemetry to read actual heading when manually aligned to goal.
     */
    final double RED_GOAL_PERPENDICULAR_HEADING_DEG = -45.0;   // Turn RIGHT (CW) toward red goal
    final double BLUE_GOAL_PERPENDICULAR_HEADING_DEG = 45.0;   // Turn LEFT (CCW) toward blue goal

    // Heading tolerance for auto-alignment (in degrees)
    // Increase if robot oscillates around target; decrease for tighter alignment
    final double ALIGN_TOLERANCE_DEG = 3.0;

    // Rotation speed during auto-alignment (0 to 1)
    final double ALIGN_ROTATION_SPEED = 0.8;

    // Proportional gain for heading correction during auto-align
    // Lower = smoother but slower, Higher = faster but may oscillate
    final double ALIGN_HEADING_KP = 0.015;

    // Declare OpMode members - 4 mecanum drive motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private IMU imu = null;

    // Alliance selection - determines which goal to align toward
    private Alliance alliance = Alliance.RED;

    // Cached IMU heading - read once per loop for consistency and performance
    private double cachedHeadingDeg = 0.0;

    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime spinUpTimer = new ElapsedTime(); // Tracks spin-up time for timeout
    int shotsFired = 0; // Tracks how many balls have been fired in current sequence
    boolean spinUpTimedOut = false; // Flag to bypass velocity check after timeout

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at IDLE. When the user requests a launch, we enter SPIN_UP to reach speed, then
     * LAUNCH/LAUNCHING to feed. After feeding completes we return to IDLE (feeding stops), but the
     * launcher motor stays at its last commanded velocity unless the driver presses X to stop.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        WAIT_BETWEEN,  // Pause between shots in multi-shot sequence
    }

    /*
     * AUTO-ALIGN STATE MACHINE
     *
     * When the driver presses L1, the robot automatically rotates to face
     * perpendicular to the goal zone border. This gives optimal ball trajectory
     * for launching.
     *
     * States:
     * - IDLE: Normal teleop control, waiting for L1 press
     * - ALIGNING: Robot is rotating toward target heading
     * - ALIGNED: Target heading reached, ready to launch (brief hold state)
     */
    private enum AlignState {
        IDLE,
        ALIGNING,
        ALIGNED,
    }

    private LaunchState launchState;
    private AlignState alignState;

    // Speed control constant
    final double DRIVE_SPEED = 1.0;  // 100% speed - full power for competition

    /*
     * STRAFE COMPENSATION MULTIPLIER
     *
     * Mecanum wheels are less efficient at strafing than driving forward/backward.
     * This is due to:
     * - Roller friction (rollers slide sideways during strafe)
     * - Weight transfer (robot "leans" during lateral movement)
     * - Floor surface interaction
     *
     * A multiplier > 1.0 compensates for this loss. Start with 1.1 and tune:
     * - If strafing feels sluggish compared to forward: increase (try 1.2)
     * - If strafing feels too sensitive: decrease (try 1.05)
     * - If robot drifts forward/backward while strafing: motors may need recalibration
     */
    final double STRAFE_MULTIPLIER = 1.1;

    // Setup variables for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        alignState = AlignState.IDLE;

        // Reset timers to start counting from initialization, not object construction
        feederTimer.reset();
        shotsFired = 0;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone/hub).
         */
        frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, PickleHardwareNames.LAUNCHER_MOTOR);
        leftFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.LEFT_FEEDER_SERVO);
        rightFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.RIGHT_FEEDER_SERVO);

        /*
         * MECANUM MOTOR DIRECTION SETUP
         *
         * For mecanum drive, motors on opposite sides spin in opposite directions to drive forward.
         * The right side motors are reversed so that positive power to all motors moves the robot forward.
         *
         * CURRENT CONFIGURATION:
         *   - Left motors (FORWARD): Spin "forward" → wheels move forward
         *   - Right motors (REVERSE): Spin "backward" mechanically → wheels move forward
         *   - Result: Robot drives forward when all motors get positive power! ✓
         *
         * IMPORTANT: Test drive your robot! If the robot spins, strafes incorrectly, or moves
         * backward when expected to go forward, you may need to swap some direction settings.
         * Common issues:
         *   - Robot spins instead of driving straight: Check left/right directions
         *   - Robot drives backward: Swap ALL directions (REVERSE↔FORWARD)
         *   - Strafing goes wrong direction: Check diagonal motor pairs
         */
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*
         * MOTOR RUN MODE OPTIONS:
         *
         * Drive motors currently use the default mode: RUN_WITHOUT_ENCODER
         * This is perfectly fine for teleop driving. Here are the three available modes:
         *
         * 1. RUN_WITHOUT_ENCODER (current default for drive motors)
         *    - What it does: Direct power control - you say "give 80% power," motor gets 80% power
         *    - Pros: Simple, responsive, instant reaction to joystick
         *    - Cons: Speed varies under load (slows down going uphill, speeds up downhill)
         *    - Best for: Teleop driving (what you're doing now) ✓
         *
         * 2. RUN_USING_ENCODER (what launcher uses below)
         *    - What it does: Closed-loop speed control - maintains constant velocity even under varying load
         *    - Pros: Consistent speed, smoother driving, maintains straight lines better
         *    - Cons: Slightly less responsive, requires encoders to be properly connected
         *    - Best for: Autonomous, or teleop if you want very smooth consistent driving
         *
         * 3. RUN_TO_POSITION
         *    - What it does: Drives to a specific encoder position then stops
         *    - Best for: Autonomous movements ("drive exactly 24 inches")
         *
         * You might want to switch drive motors to RUN_USING_ENCODER if:
         *    - Robot doesn't drive straight (one motor is weaker)
         *    - You want smoother, more consistent driving
         *    - Battery level affects driving performance too much
         */

        /*
         * Set drive motors to RUN_WITHOUT_ENCODER for maximum speed (raw power control).
         * This matches PickleAutoHolonomic for consistent performance between Auto and TeleOp.
         *
         * RUN_WITHOUT_ENCODER = direct power control, maximum speed
         * RUN_USING_ENCODER = velocity-controlled, consistent but slower
         */
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        /*
         * LAUNCHER PIDF COEFFICIENT TUNING
         *
         * These coefficients control how the launcher motor reaches and maintains target velocity.
         * Format: PIDFCoefficients(P, I, D, F)
         *
         * Current values: P=50, I=0, D=0, F=14
         *
         * WHAT EACH COEFFICIENT DOES:
         * - P (Proportional): 50
         *   Main corrective force. Higher = faster response but can cause oscillation.
         *   If motor overshoots/oscillates around target, decrease P.
         *   If motor is too slow to reach target, increase P.
         *
         * - I (Integral): 0
         *   Eliminates steady-state error over time. Usually 0 for velocity control.
         *   Only increase if motor consistently stays below target velocity.
         *
         * - D (Derivative): 0
         *   Dampens oscillation. Usually 0 for FTC velocity control.
         *   Increase slightly if you reduced P but still see oscillation.
         *
         * - F (Feedforward): 14
         *   Predictive component that provides base power for target velocity.
         *   CALCULATION FOR goBILDA 5203 (312 RPM, 537.7 CPR):
         *     Max velocity = 312/60 × 537.7 = 2,796 ticks/sec
         *     At 100% power (32767 internal units), motor runs at max velocity
         *     F = 32767 / 2796 ≈ 11.7, rounded up to 14 for margin
         *
         * TUNING PROCESS:
         * 1. Set F first using the formula: F = 32767 / max_ticks_per_sec
         * 2. Start with P=30, increase until motor reaches target quickly
         * 3. If oscillating, reduce P by 20-30%
         * 4. Only adjust I/D if problems persist
         * 5. Test with actual game pieces - loaded performance may differ
         *
         * HARDWARE-SPECIFIC: Tuned for goBILDA 5203 (312 RPM, 19.2:1) motor.
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0, 14));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * IMU INITIALIZATION
         *
         * The IMU (Inertial Measurement Unit) provides accurate heading information.
         * This is critical for the auto-align feature to know which way the robot is facing.
         *
         * IMPORTANT: Configure the IMU orientation to match how the Control Hub is mounted:
         * - LogoFacingDirection: Which way the REV logo faces (UP, DOWN, FORWARD, etc.)
         * - UsbFacingDirection: Which way the USB ports face (FORWARD, LEFT, RIGHT, etc.)
         *
         * If the robot rotates the wrong direction during alignment, swap USB direction.
         */
        try {
            imu = hardwareMap.get(IMU.class, PickleHardwareNames.IMU_NAME);
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(orientation));
        } catch (Exception e) {
            imu = null;
            telemetry.addData("IMU ERROR", e.getMessage());
        }

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("IMU", imu != null ? "Ready" : "NOT AVAILABLE");
        telemetry.addData("Alliance", alliance);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     *
     * USE THIS TIME TO:
     * 1. Select alliance color (A for Red, B for Blue)
     * 2. Verify IMU is working
     * 3. Position robot on starting tile
     */
    @Override
    public void init_loop() {
        // Alliance selection - press A for Red, B for Blue
        if (gamepad1.a) {
            alliance = Alliance.RED;
        }
        if (gamepad1.b) {
            alliance = Alliance.BLUE;
        }

        // Display alliance selection instructions
        telemetry.addData("--- ALLIANCE SELECTION ---", "");
        telemetry.addData("Press A", "for RED");
        telemetry.addData("Press B", "for BLUE");
        telemetry.addData("Current Alliance", alliance);
        telemetry.addLine();

        // Display target heading for current alliance
        double targetHeading = (alliance == Alliance.RED)
                ? RED_GOAL_PERPENDICULAR_HEADING_DEG
                : BLUE_GOAL_PERPENDICULAR_HEADING_DEG;
        telemetry.addData("Target Heading", "%.1f°", targetHeading);

        // Display current IMU heading for verification
        if (imu != null) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Current IMU Heading", "%.1f°", currentHeading);
        } else {
            telemetry.addData("IMU", "NOT AVAILABLE - alignment disabled");
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     *
     * This is the moment the match ACTUALLY begins. Use this to:
     * 1. Reset IMU yaw - makes current heading the "zero" reference
     * 2. Reset timers - so they count from match start, not init
     * 3. Reset state machines - ensure clean starting state
     *
     * WHY NOT IN init()?
     * The time between init() and start() is unpredictable (could be minutes).
     * During this time, the robot may be repositioned, rotated, or adjusted.
     * Resetting here ensures consistent behavior regardless of setup time.
     */
    @Override
    public void start() {
        // Reset IMU yaw so the robot's current facing direction becomes 0°
        // This makes auto-align headings relative to match start position
        if (imu != null) {
            imu.resetYaw();
        }

        // Reset timers to count from match start
        feederTimer.reset();
        shotsFired = 0;

        // Ensure state machines start in known states
        // (Already set in init(), but good practice to be explicit)
        launchState = LaunchState.IDLE;
        alignState = AlignState.IDLE;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * STEP 1: CACHE SENSOR READINGS
         *
         * Read sensors ONCE at the start of each loop for:
         * - Consistency: Same values used for control and telemetry
         * - Performance: I2C reads take ~2ms each, avoid redundant reads
         */
        if (imu != null) {
            cachedHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        /*
         * STEP 2: AUTO-ALIGN TO GOAL (Left Bumper - Toggle)
         *
         * Press left bumper to toggle auto-alignment on/off. When enabled, the robot
         * automatically rotates to face perpendicular to the goal zone border for
         * optimal ball launching trajectory.
         *
         * The driver can cancel alignment at any time by:
         * - Moving the right stick (manual rotation override)
         * - Pressing left bumper again (toggle off)
         */
        if (gamepad1.leftBumperWasPressed()) {
            // Toggle alignment state on button press
            if (alignState == AlignState.IDLE) {
                alignState = AlignState.ALIGNING;
            } else {
                alignState = AlignState.IDLE;
            }
        }

        // Cancel alignment if driver moves the right stick (manual override)
        if (alignState == AlignState.ALIGNING && Math.abs(gamepad1.right_stick_x) > 0.1) {
            alignState = AlignState.IDLE;
        }

        // Get rotation command from auto-align (returns 0 if not aligning)
        double alignRotation = getAlignRotation();

        /*
         * STEP 3: MECANUM DRIVE CONTROLS
         *
         * - Left stick Y-axis: Forward/backward movement
         * - Left stick X-axis: Strafing (side-to-side movement)
         * - Right stick X-axis: Rotation (turning)
         * - Auto-align rotation: Added separately (bypasses input shaping)
         *
         * Note: We negate left_stick_y because pushing forward gives negative values on gamepad.
         *
         * Driver rotation and auto-rotation are passed separately so that:
         * - Driver inputs get deadband + quadratic shaping (for fine control)
         * - Auto-rotation bypasses shaping (computed values need full precision)
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, alignRotation);

        /*
         * STEP 4: LAUNCHER CONTROL (Centralized)
         *
         * All launcher velocity control goes through the launch() state machine.
         * Button inputs set request flags, state machine decides actual velocity.
         *
         * Controls:
         * - Y button: Request spin-up (pre-heat launcher for faster shots)
         * - X button: Request stop (cancel spin-up, stop launcher)
         * - Right bumper: Request shot (spin up + feed when ready)
         */
        boolean requestSpinUp = gamepad1.y;
        boolean requestStop = gamepad1.x;
        boolean requestShot = gamepad1.rightBumperWasPressed();
        launch(requestShot, requestSpinUp, requestStop);

        /*
         * STEP 5: TELEMETRY
         */
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Launch State", launchState);

        // Show alignment status using cached heading
        telemetry.addData("Align State", alignState);
        if (imu != null) {
            double targetHeading = (alliance == Alliance.RED)
                    ? RED_GOAL_PERPENDICULAR_HEADING_DEG
                    : BLUE_GOAL_PERPENDICULAR_HEADING_DEG;
            double headingError = normalizeAngleDegrees(targetHeading - cachedHeadingDeg);
            telemetry.addData("Heading", "%.1f° → %.1f° (err: %.1f°)",
                    cachedHeadingDeg, targetHeading, headingError);
        }

        // Show multi-shot progress
        if (launchState != LaunchState.IDLE) {
            telemetry.addData("Launch Progress", "%d/%d balls", shotsFired, BALLS_PER_LAUNCH);
        } else {
            telemetry.addData("Launch Status", "READY (%d balls)", BALLS_PER_LAUNCH);
        }

        // Display all 4 motor powers in a readable format
        telemetry.addData("Front Motors", "L:%.2f  R:%.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Motors", "L:%.2f  R:%.2f", backLeftPower, backRightPower);
        telemetry.addData("Launcher Speed", launcher.getVelocity());

        /*
         * IMPORTANT: telemetry.update() must be called to push data to Driver Station
         */
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     *
     * CRITICAL SAFETY METHOD: This ensures all motors and servos are stopped
     * when the OpMode ends. Without this cleanup:
     * - Motors could keep spinning at their last commanded power
     * - Servos could remain active
     * - The robot could move unexpectedly when disabled
     *
     * This is especially important if the OpMode crashes or is stopped
     * mid-operation. Always implement proper cleanup in stop()!
     */
    @Override
    public void stop() {
        // Stop all drive motors - prevents robot from rolling away
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Stop the launcher motor - safety first with spinning mechanisms
        launcher.setVelocity(0);

        // Stop feeder servos - ensures no continued feeding action
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    /*
     * MECANUM DRIVE CONTROL
     *
     * Mecanum wheels have rollers at 45° angles that create unique force vectors.
     * By combining the power of 4 wheels, the robot can move in ANY direction:
     *
     *   - Left stick Y-axis (forward): All wheels work together for forward/backward
     *   - Left stick X-axis (strafe): Diagonal wheel pairs work together for side movement
     *   - Right stick X-axis (rotate): Left/right sides oppose for rotation
     *
     * MECANUM DRIVE FORMULA (standard configuration):
     *   Front Left  = forward + strafe + rotate
     *   Front Right = forward - strafe - rotate
     *   Back Left   = forward - strafe + rotate
     *   Back Right  = forward + strafe - rotate
     *
     * WHY THIS WORKS:
     * When strafing right (strafe > 0):
     *   - FL and BR spin forward (rollers push robot right)
     *   - FR and BL spin backward (rollers also push robot right)
     *   - All 4 wheels contribute to rightward motion!
     *
     * Speed control:
     *   - Normal mode: 70% max speed (easier to control than full speed)
     *   - Slow mode (left bumper): 30% max speed (precision movements)
     *
     * This implementation includes:
     * 1. DEADBAND - Ignores joystick drift (values < 5%)
     * 2. NORMALIZATION - Scales powers proportionally when exceeding ±1.0
     *
     * IMPORTANT: Driver inputs (forward, strafe, driverRotate) get deadband + shaping
     * for fine control. Auto-rotation bypasses shaping to preserve computed precision.
     *
     * @param forward       Driver forward/backward input (left stick Y)
     * @param strafe        Driver strafe input (left stick X)
     * @param driverRotate  Driver rotation input (right stick X) - gets shaped
     * @param autoRotate    Auto-align rotation (computed) - bypasses shaping
     */
    void mecanumDrive(double forward, double strafe, double driverRotate, double autoRotate) {
        // STEP 1: Apply SCALED deadband to eliminate stick drift with smooth transition
        // Unlike a simple deadband that creates a "jump" at the threshold,
        // scaled deadband remaps the remaining range (0.05 to 1.0) back to (0.0 to 1.0)
        // This provides seamless control from stopped to full speed
        final double DEADBAND = 0.05;  // 5% deadband threshold
        forward = applyScaledDeadband(forward, DEADBAND);
        strafe = applyScaledDeadband(strafe, DEADBAND);
        driverRotate = applyScaledDeadband(driverRotate, DEADBAND);
        // NOTE: autoRotate does NOT get deadband - it's a computed value, not joystick input

        // STEP 2: Apply INPUT SHAPING for finer low-speed control
        // Squaring the input (while preserving sign) creates a curved response:
        // - Small stick movements → very small power (precise positioning)
        // - Large stick movements → near full power (fast movement when needed)
        // This is called "quadratic scaling" and is widely used in FTC/FRC
        forward = shapeInput(forward);
        strafe = shapeInput(strafe);
        driverRotate = shapeInput(driverRotate);
        // NOTE: autoRotate does NOT get shaped - computed values need full precision!
        // Without this separation, 0.12 auto-rotate becomes 0.12² = 0.0144 (too weak!)

        // STEP 3: Combine driver rotation with auto-rotation
        // If auto-aligning, autoRotate will have a value; otherwise it's 0
        // Driver can override auto-align by moving right stick (handled in loop())
        double rotate = driverRotate + autoRotate;

        // STEP 4: Apply strafe compensation
        // Mecanum wheels strafe less efficiently due to roller friction
        // Multiplying strafe input makes lateral movement feel equal to forward movement
        strafe = strafe * STRAFE_MULTIPLIER;

        // STEP 5: Calculate raw motor powers using mecanum drive formula
        double rawFrontLeft = forward + strafe + rotate;
        double rawFrontRight = forward - strafe - rotate;
        double rawBackLeft = forward - strafe + rotate;
        double rawBackRight = forward + strafe - rotate;

        // STEP 6: Normalize to prevent clipping distortion
        // Find the maximum absolute value among all 4 motors
        // If max > 1.0, scale all down proportionally to preserve the ratio
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(rawFrontLeft), Math.abs(rawFrontRight)),
                Math.max(Math.abs(rawBackLeft), Math.abs(rawBackRight))
        ));
        rawFrontLeft /= maxPower;
        rawFrontRight /= maxPower;
        rawBackLeft /= maxPower;
        rawBackRight /= maxPower;

        // STEP 7: Apply speed multiplier
        frontLeftPower = rawFrontLeft * DRIVE_SPEED;
        frontRightPower = rawFrontRight * DRIVE_SPEED;
        backLeftPower = rawBackLeft * DRIVE_SPEED;
        backRightPower = rawBackRight * DRIVE_SPEED;

        // STEP 8: Send calculated power to all 4 wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /*
     * SCALED DEADBAND HELPER FUNCTION
     *
     * An improved deadband that eliminates the "jump" problem of simple deadbands.
     *
     * PROBLEM WITH SIMPLE DEADBAND:
     * Simple deadband creates a discontinuity - the output jumps from 0 to 0.05
     * the instant you cross the threshold. This feels jarring to drivers.
     *
     * HOW SCALED DEADBAND WORKS:
     *   - If |value| < deadband → return 0.0 (ignore noise, same as simple)
     *   - Otherwise → REMAP the range [deadband, 1.0] to [0.0, 1.0]
     *
     * Example with deadband = 0.05:
     *   - Input: 0.03  → Output: 0.0 (below threshold)
     *   - Input: 0.05  → Output: 0.0 (at threshold, smooth start)
     *   - Input: 0.10  → Output: ~0.053 (smooth ramp-up)
     *   - Input: 0.525 → Output: 0.5 (midpoint)
     *   - Input: 1.0   → Output: 1.0 (full power)
     *
     * FORMULA: output = (|value| - deadband) / (1.0 - deadband) * sign(value)
     */
    double applyScaledDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // Remap the remaining range [deadband, 1.0] to [0.0, 1.0]
        // Math.copySign preserves the original sign (positive/negative)
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    /*
     * INPUT SHAPING HELPER FUNCTION
     *
     * Currently configured for LINEAR response (input = output).
     * This provides maximum responsiveness and full power availability.
     *
     * WHY LINEAR?
     * - Maximum speed: 100% stick = 100% power (no reduction)
     * - Instant response: Great for competition driving
     * - Simpler control: What you push is what you get
     *
     * TRADE-OFF:
     * Linear response makes fine adjustments harder at low speeds.
     * If you need more precision for alignment, consider switching to
     * quadratic scaling by uncommenting the alternative implementation below.
     *
     * QUADRATIC ALTERNATIVE (for precision driving):
     * Squaring the input creates a curved response:
     *   - Input: 0.3  → Output: 0.09 (30% stick = 9% power - precise!)
     *   - Input: 0.5  → Output: 0.25 (50% stick = 25% power)
     *   - Input: 0.7  → Output: 0.49 (70% stick = 49% power)
     *   - Input: 1.0  → Output: 1.0  (100% stick = 100% power)
     *
     * To enable quadratic scaling, change the return statement to:
     *   return Math.copySign(value * value, value);
     */
    double shapeInput(double value) {
        // Linear response - maximum speed, direct control
        return value;

        // ALTERNATIVE: Quadratic scaling for finer low-speed control
        // return Math.copySign(value * value, value);
    }

    /*
     * CENTRALIZED LAUNCHER CONTROL
     *
     * All launcher velocity decisions are made here to prevent conflicts.
     * The state machine has full control over launcher motor and feeders.
     *
     * @param requestShot   Right bumper pressed - request a full launch sequence
     * @param requestSpinUp Y button held - pre-spin the launcher (faster shots)
     * @param requestStop   X button held - stop/cancel launcher
     */
    void launch(boolean requestShot, boolean requestSpinUp, boolean requestStop) {
        // Handle stop request - can cancel from any state
        if (requestStop) {
            launchState = LaunchState.IDLE;
            launcher.setVelocity(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            shotsFired = 0;
            spinUpTimedOut = false;
            return;
        }

        switch (launchState) {
            case IDLE:
                // IDLE means no feeding; launcher keeps last velocity unless X pressed.
                // Manual spin-up request (Y button) - pre-heat launcher for faster shots
                if (requestSpinUp) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                }

                // Shot request (Right bumper) - start multi-shot sequence
                if (requestShot) {
                    shotsFired = 0; // Reset counter for new sequence
                    spinUpTimedOut = false; // Reset timeout flag
                    spinUpTimer.reset(); // Start timing spin-up
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Keep launcher spinning at target velocity
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Check if we've reached minimum velocity for launch
                // OR if we've waited too long (timeout prevents stuck sequence)
                boolean velocityReady = launcher.getVelocity() > LAUNCHER_MIN_VELOCITY;
                boolean timedOut = spinUpTimer.seconds() > SPIN_UP_TIMEOUT_SECONDS;
                if (velocityReady || timedOut) {
                    spinUpTimedOut = timedOut && !velocityReady; // Flag to bypass velocity check
                    launchState = LaunchState.LAUNCH;
                }

                telemetry.addData("Spin-up Progress", "%.2fs | %.0f / %.0f",
                    spinUpTimer.seconds(), launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
                break;

            case LAUNCH:
                // Re-check velocity before each shot to prevent weak launches
                // BUT skip check if we already timed out (fire anyway)
                if (!spinUpTimedOut && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                    spinUpTimer.reset(); // Reset timeout for velocity recovery
                    launchState = LaunchState.SPIN_UP;
                    break;
                }

                // Log pre-shot velocity for trajectory analysis
                telemetry.addData(String.format("Ball %d Pre-shot Velocity", shotsFired + 1),
                    "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);

                // Ensure launcher is at target velocity before feeding
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Activate feeders to push ball into launcher
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                spinUpTimedOut = false; // Reset flag after using it
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // Keep launcher at speed while feeding
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Wait for feed time to complete
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    shotsFired++;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);

                    // Check if we've fired all balls
                    if (shotsFired >= BALLS_PER_LAUNCH) {
                        launchState = LaunchState.IDLE;
                        shotsFired = 0;
                    } else {
                        // More balls to fire - enter wait state before next shot
                        feederTimer.reset();
                        launchState = LaunchState.WAIT_BETWEEN;
                    }
                }
                break;

            case WAIT_BETWEEN:
                // Keep launcher spinning during pause
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Track velocity recovery between shots
                telemetry.addData("Velocity Recovery", "%.0f / %.0f",
                    launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);

                // Wait for interval between shots (feeders are stopped)
                if (feederTimer.seconds() > SHOT_INTERVAL_SECONDS) {
                    // Go to LAUNCH which will re-check velocity before firing
                    launchState = LaunchState.LAUNCH;
                }
                break;
        }
    }

    /*
     * GET AUTO-ALIGN ROTATION COMMAND
     *
     * This method calculates the rotation power needed to face perpendicular to the goal zone.
     * Instead of directly applying motor powers (which would be overwritten by mecanumDrive),
     * it RETURNS the rotation value to be passed into mecanumDrive().
     *
     * ALGORITHM:
     * 1. Get target heading based on alliance (135° for Red, 45° for Blue)
     * 2. Use cached IMU heading (read once per loop for consistency)
     * 3. Calculate heading error (difference between target and current)
     * 4. Apply proportional control: rotate_power = Kp * error
     * 5. Clamp power to max alignment speed
     * 6. Return rotation power (not apply directly!)
     *
     * PROPORTIONAL CONTROL (P-controller):
     * - Error = Target - Current
     * - Output = Kp * Error
     * - When far from target: high error → high rotation power
     * - When close to target: low error → low rotation power (prevents overshoot)
     *
     * @return rotation power to pass to mecanumDrive(), or 0.0 if not aligning
     */
    double getAlignRotation() {
        // Only align if in ALIGNING state and IMU is available
        if (alignState != AlignState.ALIGNING || imu == null) {
            return 0.0;
        }

        // Get target heading based on alliance
        double targetHeadingDeg = (alliance == Alliance.RED)
                ? RED_GOAL_PERPENDICULAR_HEADING_DEG
                : BLUE_GOAL_PERPENDICULAR_HEADING_DEG;

        // Use cached heading (read once at start of loop)
        double headingErrorDeg = normalizeAngleDegrees(targetHeadingDeg - cachedHeadingDeg);

        // Check if we're close enough - aligned!
        if (Math.abs(headingErrorDeg) <= ALIGN_TOLERANCE_DEG) {
            alignState = AlignState.ALIGNED;
            return 0.0;
        }

        // Apply proportional control for smooth rotation
        // Positive error (target > current) → need to rotate counter-clockwise → negative rotation
        // Negative error (target < current) → need to rotate clockwise → positive rotation
        // Note: In our mecanum formula, positive rotate = clockwise, so we negate
        double rotatePower = -headingErrorDeg * ALIGN_HEADING_KP;

        // Clamp to maximum alignment speed
        rotatePower = clamp(rotatePower, -ALIGN_ROTATION_SPEED, ALIGN_ROTATION_SPEED);

        // Ensure minimum power to overcome static friction
        final double MIN_ROTATION_POWER = 0.12;
        if (Math.abs(rotatePower) < MIN_ROTATION_POWER && Math.abs(headingErrorDeg) > ALIGN_TOLERANCE_DEG) {
            rotatePower = Math.copySign(MIN_ROTATION_POWER, rotatePower);
        }

        // Return rotation power - mecanumDrive() will apply it along with translation
        return rotatePower;
    }

    /*
     * NORMALIZE ANGLE TO -180 TO 180 DEGREES
     *
     * This helper function normalizes any angle to the range [-180, 180].
     * This is crucial for heading error calculations to ensure we always
     * take the shortest rotation path.
     *
     * Examples:
     * - Input: 270° → Output: -90° (rotate 90° clockwise instead of 270° CCW)
     * - Input: -270° → Output: 90° (rotate 90° CCW instead of 270° CW)
     * - Input: 45° → Output: 45° (no change)
     *
     * @param angleDeg Angle in degrees (any value)
     * @return Normalized angle in range [-180, 180]
     */
    double normalizeAngleDegrees(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    /*
     * CLAMP VALUE TO RANGE
     *
     * @param value Value to clamp
     * @param min   Minimum allowed value
     * @param max   Maximum allowed value
     * @return Value clamped to [min, max]
     */
    double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
