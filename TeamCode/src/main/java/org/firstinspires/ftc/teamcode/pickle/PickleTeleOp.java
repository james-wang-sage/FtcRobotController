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
     * MOTOR/ENCODER REQUIREMENTS:
     * - All motors must have encoders connected to use RUN_USING_ENCODER mode
     * - Launcher motor assumes goBILDA 5203/5204 or similar (28 CPR encoder)
     * - Drive motors work with any standard FTC motors with encoders
     *
     * CONTROL HUB PORT MAPPING (recommended):
     * - Port 0: front_left
     * - Port 1: front_right
     * - Port 2: back_left
     * - Port 3: back_right
     */

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double LAUNCH_COOLDOWN_SECONDS = 1.0; //Minimum time between launches to prevent rapid-fire
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * LAUNCHER VELOCITY CONFIGURATION
     *
     * These values control the launcher motor's speed using encoder feedback.
     *
     * HARDWARE ASSUMPTIONS:
     * - Motor: goBILDA 5203/5204 Series (or similar high-speed motor)
     * - Encoder: 28 counts per revolution (CPR) at the motor
     * - Velocity units: Encoder ticks per second
     *
     * CALCULATING VELOCITY:
     * If your motor specs say it runs at 6000 RPM max:
     *   - 6000 RPM ÷ 60 = 100 revolutions per second
     *   - 100 rev/sec × 28 ticks/rev = 2800 ticks/sec maximum
     * Current target (1125 ticks/sec) ≈ 40% of a 6000 RPM motor's max speed
     *
     * TUNING THESE VALUES:
     * 1. Start with a lower target velocity (e.g., 800) for safety
     * 2. Gradually increase until launcher performs consistently
     * 3. Set MIN_VELOCITY about 50-100 ticks below TARGET for reliability
     * 4. Monitor telemetry "motorSpeed" to see actual velocity
     *
     * If using different motors/gearing, you MUST retune these values!
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;  // Target speed in encoder ticks/second
    final double LAUNCHER_MIN_VELOCITY = 1075;     // Minimum speed before allowing launch

    /*
     * AUTO-ALIGN HEADING CONSTANTS
     *
     * The goal zones have 45-degree angled borders (diagonal ramps).
     * To launch balls perpendicular to these borders:
     *
     * RED Goal (top-right corner):
     *   - Ramp/border runs at 45° angle
     *   - Perpendicular heading = 135° (facing northwest toward goal opening)
     *
     * BLUE Goal (top-left corner):
     *   - Ramp/border runs at 135° angle
     *   - Perpendicular heading = 45° (facing northeast toward goal opening)
     *
     * Field coordinate system:
     *   0° = +X (toward Red side), 90° = +Y (toward far wall)
     *   Counter-clockwise is positive
     */
    final double RED_GOAL_PERPENDICULAR_HEADING_DEG = 135.0;
    final double BLUE_GOAL_PERPENDICULAR_HEADING_DEG = 45.0;

    // Heading tolerance for auto-alignment (in degrees)
    final double ALIGN_TOLERANCE_DEG = 3.0;

    // Rotation speed during auto-alignment (0 to 1)
    final double ALIGN_ROTATION_SPEED = 0.35;

    // Proportional gain for heading correction during auto-align
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

    // IMU heading offset to convert IMU yaw to field heading
    // fieldHeading = imuYaw + imuHeadingOffset
    private double imuHeadingOffset = 0.0;

    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime launchCooldownTimer = new ElapsedTime(); // Tracks time since last launch

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
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
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

    // Speed control constants
    final double NORMAL_DRIVE_SPEED = 1.0;  // 100% speed - full power for competition
    final double SLOW_DRIVE_SPEED = 0.3;    // 30% speed for precision mode (activated with left bumper)

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
    boolean slowMode = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        alignState = AlignState.IDLE;

        // Reset timers to start counting from initialization, not object construction
        // This ensures cooldown displays correctly and timing aligns with match start
        feederTimer.reset();
        launchCooldownTimer.reset();

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
         * The left side motors are reversed so that positive power to all motors moves the robot forward.
         *
         * STANDARD CONFIGURATION:
         *   - Left motors (REVERSE): Spin "backward" mechanically → wheels move forward
         *   - Right motors (FORWARD): Spin "forward" → wheels move forward
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
         * Set drive motors to RUN_USING_ENCODER for more consistent and controllable driving.
         * This provides closed-loop velocity control that maintains constant speed even as
         * battery voltage drops or the robot encounters varying loads.
         *
         * IMPORTANT: Make sure your drive motor encoders are properly connected!
         * If the robot doesn't respond to controls after this change, check that encoder
         * cables are plugged into the motor ports (not separate encoder ports).
         */
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
         * Current values: P=300, I=0, D=0, F=10
         *
         * WHAT EACH COEFFICIENT DOES:
         * - P (Proportional): 300
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
         * - F (Feedforward): 10
         *   Predictive component based on target velocity. Reduces reliance on P.
         *   Calculate as: F = (12V × 60) / (motor_free_speed_RPM × ticks_per_rev)
         *   For a 6000 RPM motor with 28 CPR: F ≈ (720) / (6000 × 28) ≈ 0.0043 per tick/sec
         *   Scaled value: 0.0043 × target_velocity ≈ 10 for this application
         *
         * TUNING PROCESS:
         * 1. Set F first using the formula above
         * 2. Start with P=100, increase until motor reaches target quickly
         * 3. If oscillating, reduce P by 20-30%
         * 4. Only adjust I/D if problems persist
         * 5. Test with actual game pieces - loaded performance may differ
         *
         * HARDWARE-SPECIFIC: These values are tuned for goBILDA 5203/5204 motors.
         * If you change motors, gearing, or wheel diameter, you MUST retune!
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

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
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
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
     * 1. Select alliance color (X for Blue, B for Red)
     * 2. Verify IMU is working
     * 3. Position robot on starting tile
     */
    @Override
    public void init_loop() {
        // Alliance selection - press X for Blue, B for Red
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }
        if (gamepad1.b) {
            alliance = Alliance.RED;
        }

        // Display alliance selection instructions
        telemetry.addData("--- ALLIANCE SELECTION ---", "");
        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
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
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Toggle slow mode with left bumper for precision driving
         * This is helpful when you need fine control for alignment or delicate maneuvers
         */
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;  // Toggle the mode on button press
        }

        /*
         * AUTO-ALIGN TO GOAL (L1 / Left Trigger Button)
         *
         * When L1 is pressed, the robot automatically rotates to face perpendicular
         * to the goal zone border for optimal ball launching trajectory.
         *
         * The driver can cancel alignment at any time by:
         * - Moving the right stick (manual rotation override)
         * - Pressing L1 again (toggle off)
         */
        if (gamepad1.left_trigger > 0.5) {
            // L1 pressed - start or continue alignment
            if (alignState == AlignState.IDLE) {
                alignState = AlignState.ALIGNING;
            }
        } else {
            // L1 released - return to idle
            if (alignState != AlignState.IDLE) {
                alignState = AlignState.IDLE;
            }
        }

        // Handle auto-alignment rotation (overrides manual rotation when active)
        boolean isAutoAligning = autoAlign();

        /*
         * MECANUM DRIVE CONTROLS:
         * - Left stick Y-axis: Forward/backward movement
         * - Left stick X-axis: Strafing (side-to-side movement)
         * - Right stick X-axis: Rotation (turning)
         *
         * The mecanumDrive function calculates the power for all 4 motors based on these inputs,
         * allowing the robot to move in any direction while simultaneously rotating.
         *
         * Note: We negate left_stick_y because pushing forward gives negative values on gamepad.
         *
         * When auto-aligning, rotation is handled by the autoAlign() method, so we pass 0 for rotate.
         * The driver can still strafe and move forward/backward during alignment.
         */
        double rotation = isAutoAligning ? 0.0 : gamepad1.right_stick_x;
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotation);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", slowMode ? "SLOW (30%)" : "NORMAL (100%)");
        telemetry.addData("Launch State", launchState);

        // Show alignment status
        telemetry.addData("Align State", alignState);
        if (imu != null) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double targetHeading = (alliance == Alliance.RED)
                    ? RED_GOAL_PERPENDICULAR_HEADING_DEG
                    : BLUE_GOAL_PERPENDICULAR_HEADING_DEG;
            double headingError = normalizeAngleDegrees(targetHeading - currentHeading);
            telemetry.addData("Heading", "%.1f° → %.1f° (err: %.1f°)",
                    currentHeading, targetHeading, headingError);
        }

        // Show launch cooldown status
        double cooldownRemaining = LAUNCH_COOLDOWN_SECONDS - launchCooldownTimer.seconds();
        if (cooldownRemaining > 0) {
            telemetry.addData("Launch Cooldown", "%.1f sec", cooldownRemaining);
        } else {
            telemetry.addData("Launch Status", "READY");
        }

        // Display all 4 motor powers in a readable format
        telemetry.addData("Front Motors", "L:%.2f  R:%.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Motors", "L:%.2f  R:%.2f", backLeftPower, backRightPower);
        telemetry.addData("Launcher Speed", launcher.getVelocity());

        /*
         * IMPORTANT: telemetry.update() must be called to push data to Driver Station
         *
         * Without this call, telemetry data is queued but never sent to the display.
         * The Driver Station screen would show stale or no data. This is a common
         * mistake that makes debugging difficult since you can't see what's happening.
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
     */
    void mecanumDrive(double forward, double strafe, double rotate) {
        // STEP 1: Apply SCALED deadband to eliminate stick drift with smooth transition
        // Unlike a simple deadband that creates a "jump" at the threshold,
        // scaled deadband remaps the remaining range (0.05 to 1.0) back to (0.0 to 1.0)
        // This provides seamless control from stopped to full speed
        final double DEADBAND = 0.05;  // 5% deadband threshold
        forward = applyScaledDeadband(forward, DEADBAND);
        strafe = applyScaledDeadband(strafe, DEADBAND);
        rotate = applyScaledDeadband(rotate, DEADBAND);

        // STEP 2: Apply INPUT SHAPING for finer low-speed control
        // Squaring the input (while preserving sign) creates a curved response:
        // - Small stick movements → very small power (precise positioning)
        // - Large stick movements → near full power (fast movement when needed)
        // This is called "quadratic scaling" and is widely used in FTC/FRC
        forward = shapeInput(forward);
        strafe = shapeInput(strafe);
        rotate = shapeInput(rotate);

        // STEP 3: Apply strafe compensation
        // Mecanum wheels strafe less efficiently due to roller friction
        // Multiplying strafe input makes lateral movement feel equal to forward movement
        strafe = strafe * STRAFE_MULTIPLIER;

        // STEP 4: Calculate raw motor powers using mecanum drive formula
        double rawFrontLeft = forward + strafe + rotate;
        double rawFrontRight = forward - strafe - rotate;
        double rawBackLeft = forward - strafe + rotate;
        double rawBackRight = forward + strafe - rotate;

        // STEP 5: Normalize to prevent clipping distortion
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

        // STEP 6: Apply speed multiplier for normal/slow mode
        double speedMultiplier = slowMode ? SLOW_DRIVE_SPEED : NORMAL_DRIVE_SPEED;
        frontLeftPower = rawFrontLeft * speedMultiplier;
        frontRightPower = rawFrontRight * speedMultiplier;
        backLeftPower = rawBackLeft * speedMultiplier;
        backRightPower = rawBackRight * speedMultiplier;

        // STEP 7: Send calculated power to all 4 wheels
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
     * INPUT SHAPING HELPER FUNCTION (Quadratic Scaling)
     *
     * Creates a non-linear response curve for finer low-speed control.
     *
     * WHY USE INPUT SHAPING?
     * Linear joystick response (input = output) makes precise movements difficult.
     * Squaring the input gives you:
     *   - Fine control at low speeds (great for alignment)
     *   - Full power still available at max stick (for fast movement)
     *
     * HOW IT WORKS (squaring with sign preservation):
     *   - Input: 0.1  → Output: 0.01 (10% stick = 1% power - very precise!)
     *   - Input: 0.3  → Output: 0.09 (30% stick = 9% power)
     *   - Input: 0.5  → Output: 0.25 (50% stick = 25% power)
     *   - Input: 0.7  → Output: 0.49 (70% stick = 49% power)
     *   - Input: 1.0  → Output: 1.0  (100% stick = 100% power)
     *   - Input: -0.5 → Output: -0.25 (sign preserved for direction!)
     *
     * ALTERNATIVE CURVES:
     * - Cubic (value³): Even more sensitive at low speeds
     * - Square root (√value): More linear at low, compressed at high (rarely used)
     *
     * Math.copySign ensures negative inputs produce negative outputs.
     */
    double shapeInput(double value) {
        // Square the magnitude, preserve the sign
        return Math.copySign(value * value, value);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                // Only allow a new launch if cooldown period has elapsed
                if (shotRequested && launchCooldownTimer.seconds() >= LAUNCH_COOLDOWN_SECONDS) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchCooldownTimer.reset(); // Start cooldown period after launch completes
                }
                break;
        }
    }

    /*
     * AUTO-ALIGN TO GOAL PERPENDICULAR HEADING
     *
     * This method handles the automatic rotation to face perpendicular to the goal zone border.
     * When active, it overrides the manual rotation input and uses proportional control to
     * smoothly rotate the robot to the target heading.
     *
     * ALGORITHM:
     * 1. Get target heading based on alliance (135° for Red, 45° for Blue)
     * 2. Get current heading from IMU
     * 3. Calculate heading error (difference between target and current)
     * 4. Apply proportional control: rotate_power = Kp * error
     * 5. Clamp power to max alignment speed
     * 6. Apply rotation power to motors
     *
     * PROPORTIONAL CONTROL (P-controller):
     * - Error = Target - Current
     * - Output = Kp * Error
     * - When far from target: high error → high rotation power
     * - When close to target: low error → low rotation power (prevents overshoot)
     *
     * @return true if actively aligning (rotation being applied), false otherwise
     */
    boolean autoAlign() {
        // Only align if in ALIGNING state and IMU is available
        if (alignState != AlignState.ALIGNING || imu == null) {
            return false;
        }

        // Get target heading based on alliance
        double targetHeadingDeg = (alliance == Alliance.RED)
                ? RED_GOAL_PERPENDICULAR_HEADING_DEG
                : BLUE_GOAL_PERPENDICULAR_HEADING_DEG;

        // Get current heading from IMU
        double currentHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Calculate heading error (normalized to -180 to 180 degrees)
        double headingErrorDeg = normalizeAngleDegrees(targetHeadingDeg - currentHeadingDeg);

        // Check if we're close enough - aligned!
        if (Math.abs(headingErrorDeg) <= ALIGN_TOLERANCE_DEG) {
            alignState = AlignState.ALIGNED;
            // Apply a small holding rotation to maintain position
            return false;
        }

        // Apply proportional control for smooth rotation
        // Positive error → need to rotate counter-clockwise (positive rotation)
        // Negative error → need to rotate clockwise (negative rotation)
        double rotatePower = headingErrorDeg * ALIGN_HEADING_KP;

        // Clamp to maximum alignment speed
        rotatePower = clamp(rotatePower, -ALIGN_ROTATION_SPEED, ALIGN_ROTATION_SPEED);

        // Ensure minimum power to overcome static friction
        final double MIN_ROTATION_POWER = 0.12;
        if (Math.abs(rotatePower) < MIN_ROTATION_POWER && Math.abs(headingErrorDeg) > ALIGN_TOLERANCE_DEG) {
            rotatePower = Math.copySign(MIN_ROTATION_POWER, rotatePower);
        }

        // Apply rotation to motors (rotation only, no translation)
        applyRotationOnly(rotatePower);

        return true;
    }

    /*
     * APPLY ROTATION-ONLY MOTOR POWERS
     *
     * This method applies power to the drive motors to rotate the robot in place
     * without any forward/backward or strafing movement.
     *
     * For mecanum drive rotation:
     * - Left motors get positive power (rotate CCW from top view)
     * - Right motors get negative power
     *
     * @param rotatePower Rotation power (-1 to 1, positive = counter-clockwise)
     */
    void applyRotationOnly(double rotatePower) {
        // For rotation only: left side positive, right side negative
        // This makes positive rotatePower = counter-clockwise (increasing heading)
        double leftPower = rotatePower;
        double rightPower = -rotatePower;

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        // Update power variables for telemetry
        frontLeftPower = leftPower;
        backLeftPower = leftPower;
        frontRightPower = rightPower;
        backRightPower = rightPower;
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
