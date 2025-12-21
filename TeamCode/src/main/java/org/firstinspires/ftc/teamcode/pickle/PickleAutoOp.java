/*
 * Copyright (c) 2025 Base 10 Assets, LLC
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
 * Neither the name of NAME nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pickle.config.PickleHardwareNames;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.pickle.field.Alliance;
import org.firstinspires.ftc.teamcode.pickle.field.DecodeField;

import java.util.List;

/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels," and two servos
 * which feed that launcher.
 *
 * STRATEGY (launcher-only / no intake):
 * - Drive away from the starting corner and into a known "close shot" pose near the goal
 * - Optionally use the GOAL AprilTag for final alignment (ignore Obelisk/motif tags)
 * - Launch 3 preloaded artifacts with a consistent cadence
 * - End in a TeleOp-friendly position (do not block partner paths)
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name="PickleAutoOp", group="StarterBot")
@Disabled
public class PickleAutoOp extends OpMode
{

    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    final double TIME_BETWEEN_SHOTS = 2;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Track width is used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double DRIVE_SPEED = 0.8;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    /*
     * ========== AUTONOMOUS TUNING VALUES ==========
     *
     * These values must be tuned on a real field. They are intentionally easy-to-find constants
     * because early autonomous success comes from repeatability and quick iteration.
     *
     * IMPORTANT: With the current hardware, we do NOT have a reliable way to detect other robots.
     * Collision avoidance must be handled by:
     * - pre-match coordination (who uses the goal lane)
     * - lane separation (non-intersecting paths)
     * - optional start delay (stagger timing)
     */

    /*
     * Optional delay at the start of autonomous. This is useful when coordinating with a partner
     * so both robots do not try to enter the goal area at the same time.
     *
     * Set to 0 to disable.
     */
    final double START_DELAY_SECONDS = 0.0;

    /*
     * Drive away from the starting corner so we are clear of the wall/traffic before turning.
     * Positive distance = forward (as defined by motor direction setup).
     */
    final double DRIVE_CLEAR_START_IN = 18.0;

    /*
     * After clearing the start, rotate toward the goal. Sign is mirrored by alliance.
     * Existing code convention:
     * - RED  uses +45 degrees
     * - BLUE uses -45 degrees
     */
    final double TURN_TOWARD_GOAL_DEG = 45.0;

    /*
     * Drive from the "cleared start" position to a close shooting position near the goal.
     * This should put the goal generally in view and within a short AprilTag alignment distance.
     */
    final double DRIVE_TO_CLOSE_SHOT_IN = 36.0;

    /*
     * Final parking move after shooting (TeleOp-ready). Negative drives backward.
     * Tune this so you are out of the goal lane and not blocking your partner.
     */
    final double PARK_BACKUP_IN = -12.0;

    /*
     * AprilTag-based "final align" tuning. This is deliberately conservative:
     * - short timeout so we don't waste all of auto if the goal tag is not visible
     * - small incremental nudges using encoders so behavior is predictable
     */
    final double GOAL_ALIGN_MAX_SECONDS = 1.5;
    final double GOAL_ALIGN_DESIRED_RANGE_IN = 10.0;
    final double GOAL_ALIGN_RANGE_TOL_IN = 2.0;
    final double GOAL_ALIGN_BEARING_TOL_DEG = 2.0;
    final double GOAL_ALIGN_TURN_STEP_DEG = 5.0;
    final double GOAL_ALIGN_DRIVE_STEP_IN = 3.0;

    /*
     * If the robot turns the wrong way during tag alignment, flip this.
     * (Different camera mounting orientations can invert the observed bearing.)
     */
    final boolean INVERT_TAG_TURN = false;

    /*
     * ========== SAFETY TIMEOUTS ==========
     *
     * Maximum time allowed for any single drive or rotate operation.
     * If the robot hasn't reached its target position within this time,
     * the operation will complete anyway to prevent hanging.
     */
    final double DRIVE_TIMEOUT_SECONDS = 5.0;
    final double ROTATE_TIMEOUT_SECONDS = 4.0;

    /*
     * ========== IMU CONFIGURATION ==========
     *
     * The IMU provides accurate heading information independent of wheel slippage.
     * Set USE_IMU_FOR_ROTATION to false to fall back to encoder-only rotation
     * if the IMU is not available or not working.
     */
    final boolean USE_IMU_FOR_ROTATION = true;
    final double IMU_HEADING_TOLERANCE_DEG = 2.0;

    int shotsToFire = 3; //The number of shots to fire in this auto.

    double robotRotationAngle = 45;

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime operationTimer = new ElapsedTime(); // Timeout protection for drive/rotate

    // Declare OpMode members - 4 mecanum drive motors (matching PickleTeleOp)
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // IMU for accurate heading control (falls back to encoders if unavailable)
    private IMU imu = null;
    private boolean imuAvailable = false;
    private double targetHeadingDeg = 0.0; // Used for IMU-based rotation

    /*
     * AprilTag vision components. The AprilTagProcessor handles the detection of AprilTags
     * in the camera frame, while the VisionPortal manages the camera hardware and processing
     * pipeline. AprilTags can be used for robot localization and navigation during autonomous.
     */
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into PREPARE then LAUNCH.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits."
     */
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        START_DELAY,
        DRIVE_CLEAR_START,
        TURN_TOWARD_GOAL,
        DRIVE_TO_CLOSE_SHOT,
        ALIGN_WITH_GOAL_TAG,
        REQUEST_SHOT,
        WAIT_FOR_SHOT,
        PARK_FOR_TELEOP,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    /*
     * Small internal alignment state. We keep it separate from AutonomousState to avoid creating
     * lots of tiny "ALIGN_TURNING", "ALIGN_DRIVING", etc. states.
     */
    private enum GoalAlignAction {
        NONE,
        TURN_NUDGE,
        DRIVE_NUDGE,
    }

    private GoalAlignAction goalAlignAction = GoalAlignAction.NONE;
    private double goalAlignNudgeAngleDeg = 0.0;
    private double goalAlignNudgeDistanceIn = 0.0;

    /*
     * Alliance selection - now uses the shared Alliance enum from the field package.
     * This allows consistent alliance handling across all OpModes and enables access
     * to alliance-specific field positions via DecodeField.getGoalCenter(alliance), etc.
     */
    private Alliance alliance = Alliance.RED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.START_DELAY;
        launchState = LaunchState.IDLE;
        goalAlignAction = GoalAlignAction.NONE;


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         *
         * REQUIRED HARDWARE CONFIGURATION (must match PickleTeleOp):
         * - "front_left"    : DcMotor (mecanum drive motor, front left)
         * - "front_right"   : DcMotor (mecanum drive motor, front right)
         * - "back_left"     : DcMotor (mecanum drive motor, back left)
         * - "back_right"    : DcMotor (mecanum drive motor, back right)
         * - "launcher"      : DcMotorEx (high-speed launcher motor with encoder)
         * - "left_feeder"   : CRServo (continuous rotation servo)
         * - "right_feeder"  : CRServo (continuous rotation servo)
         */
        frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, PickleHardwareNames.LAUNCHER_MOTOR);
        leftFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.LEFT_FEEDER_SERVO);
        rightFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.RIGHT_FEEDER_SERVO);

        /*
         * MECANUM MOTOR DIRECTION SETUP (matching PickleTeleOp)
         *
         * For mecanum drive, motors on opposite sides spin in opposite directions to drive forward.
         * The left side motors are reversed so that positive power to all motors moves the robot forward.
         */
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, and it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Here we set the aforementioned PID coefficients. You shouldn't have to do this for any
         * other motors on this robot.
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Initialize the IMU for accurate heading control.
         * The IMU is built into the REV Control Hub and provides gyroscope data.
         * If initialization fails, we fall back to encoder-based rotation.
         */
        initIMU();

        /*
         * Initialize the AprilTag processor and VisionPortal.
         * The "easy" methods create default configurations that work well for most use cases.
         * The AprilTag processor will automatically detect any visible AprilTags and provide
         * pose estimation data (position and orientation relative to the camera).
         */
        initAprilTag();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        /*
         * We also set the servo power to 0 here to make sure that the servo controller is booted
         * up and ready to go.
         */
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);


        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Goal AprilTag ID", DecodeField.getGoalAprilTagId(alliance));
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        // Reset any timers that should start counting at the beginning of autonomous.
        stateTimer.reset();
        driveTimer.reset();
        shotTimer.reset();
        feederTimer.reset();

        // Reset stateful variables in case we INIT/START multiple times without power-cycling.
        shotsToFire = 3;
        goalAlignAction = GoalAlignAction.NONE;
        resetDriveEncoders();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState){
            /*
             * Optional wait at the beginning of auto. This is a simple and reliable way to avoid
             * collisions with a partner robot when both teams want to use the same scoring lane.
             */
            case START_DELAY:
                if (stateTimer.seconds() >= START_DELAY_SECONDS) {
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for drive operation
                    stateTimer.reset();
                    autonomousState = AutonomousState.DRIVE_CLEAR_START;
                }
                break;

            /*
             * Step 1: Drive away from the starting corner.
             * Goal: get clear of the wall and initial traffic before turning toward the goal.
             */
            case DRIVE_CLEAR_START:
                if (drive(DRIVE_SPEED, DRIVE_CLEAR_START_IN, DistanceUnit.INCH, 0.25)) {
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for rotate operation
                    autonomousState = AutonomousState.TURN_TOWARD_GOAL;
                }
                break;

            /*
             * Step 2: Turn toward the goal. We mirror the sign based on alliance so the path is
             * symmetrical for red vs blue.
             */
            case TURN_TOWARD_GOAL:
                robotRotationAngle = (alliance == Alliance.RED) ? TURN_TOWARD_GOAL_DEG : -TURN_TOWARD_GOAL_DEG;
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 0.25)) {
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for drive operation
                    autonomousState = AutonomousState.DRIVE_TO_CLOSE_SHOT;
                }
                break;

            /*
             * Step 3: Drive to a close, repeatable shooting position near the goal.
             * We use a slower approach speed near the goal area to reduce impact risk.
             */
            case DRIVE_TO_CLOSE_SHOT:
                if (drive(0.30, DRIVE_TO_CLOSE_SHOT_IN, DistanceUnit.INCH, 0.25)) {
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for alignment operations
                    stateTimer.reset();
                    goalAlignAction = GoalAlignAction.NONE;
                    autonomousState = AutonomousState.ALIGN_WITH_GOAL_TAG;
                }
                break;

            /*
             * Step 4 (optional): Final alignment using the GOAL AprilTag only.
             *
             * Important details:
             * - We intentionally ignore "Obelisk" tags because they are not intended for localization.
             * - If the goal tag is not visible quickly, we stop trying and shoot from our dead-reckoned pose.
             * - Alignment is implemented as small "nudge" moves using encoders for predictability.
             */
            case ALIGN_WITH_GOAL_TAG: {
                if (stateTimer.seconds() > GOAL_ALIGN_MAX_SECONDS) {
                    resetDriveEncoders();
                    driveTimer.reset();
                    goalAlignAction = GoalAlignAction.NONE;
                    autonomousState = AutonomousState.REQUEST_SHOT;
                    break;
                }

                AprilTagDetection goalTag = getGoalDetection();
                if (goalTag == null) {
                    // Do nothing: keep the robot still and keep looking for the goal tag.
                    // If you want a "search", do it with caution (slow sweep) to avoid collisions.
                    break;
                }

                if (goalAlignAction == GoalAlignAction.TURN_NUDGE) {
                    if (rotate(ROTATE_SPEED, goalAlignNudgeAngleDeg, AngleUnit.DEGREES, 0.15)) {
                        resetDriveEncoders();
                        driveTimer.reset();
                        goalAlignAction = GoalAlignAction.NONE;
                    }
                    break;
                }

                if (goalAlignAction == GoalAlignAction.DRIVE_NUDGE) {
                    if (drive(0.25, goalAlignNudgeDistanceIn, DistanceUnit.INCH, 0.15)) {
                        resetDriveEncoders();
                        driveTimer.reset();
                        goalAlignAction = GoalAlignAction.NONE;
                    }
                    break;
                }

                // Decide what nudge to perform next based on the current detection.
                double bearingDeg = goalTag.ftcPose.bearing;
                double rangeIn = goalTag.ftcPose.range;

                /*
                 * The SDK's bearing convention depends on camera orientation. The intent is:
                 * - If the tag is left of center, rotate left to center it.
                 * - If the tag is right of center, rotate right to center it.
                 *
                 * If you see the robot turning away from the tag, flip INVERT_TAG_TURN above.
                 */
                if (Math.abs(bearingDeg) > GOAL_ALIGN_BEARING_TOL_DEG) {
                    double nudge = -Math.copySign(GOAL_ALIGN_TURN_STEP_DEG, bearingDeg);
                    if (INVERT_TAG_TURN) {
                        nudge = -nudge;
                    }
                    goalAlignNudgeAngleDeg = nudge;
                    goalAlignAction = GoalAlignAction.TURN_NUDGE;
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for nudge
                    break;
                }

                double rangeErrorIn = rangeIn - GOAL_ALIGN_DESIRED_RANGE_IN;
                if (Math.abs(rangeErrorIn) > GOAL_ALIGN_RANGE_TOL_IN) {
                    double nudge = Math.copySign(GOAL_ALIGN_DRIVE_STEP_IN, rangeErrorIn);
                    // Positive nudge drives forward if we are too far; negative drives backward if too close.
                    goalAlignNudgeDistanceIn = nudge;
                    goalAlignAction = GoalAlignAction.DRIVE_NUDGE;
                    resetDriveEncoders();
                    driveTimer.reset();
                    operationTimer.reset(); // Start timeout tracking for nudge
                    break;
                }

                // We are close enough in both heading and distance: proceed to shooting.
                resetDriveEncoders();
                driveTimer.reset();
                goalAlignAction = GoalAlignAction.NONE;
                autonomousState = AutonomousState.REQUEST_SHOT;
                break;
            }

            /*
             * Step 5: Shoot the 3 preloaded artifacts.
             *
             * Implementation detail:
             * - REQUEST_SHOT requests one shot (kick off launch state machine).
             * - WAIT_FOR_SHOT keeps calling launch(false) until that shot finishes.
             * - When a shot finishes, we decrement shotsToFire and repeat until we hit 0.
             */
            case REQUEST_SHOT:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_SHOT;
                break;

            case WAIT_FOR_SHOT:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.REQUEST_SHOT;
                    } else {
                        launcher.setVelocity(0);
                        resetDriveEncoders();
                        driveTimer.reset();
                        operationTimer.reset(); // Start timeout tracking for park operation
                        autonomousState = AutonomousState.PARK_FOR_TELEOP;
                    }
                }
                break;

            /*
             * Step 6: Park / reposition for TeleOp.
             * This is intentionally a simple, predictable move.
             */
            case PARK_FOR_TELEOP:
                if (drive(0.35, PARK_BACKUP_IN, DistanceUnit.INCH, 0.25)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("IMU", imuAvailable ? "Active" : "Fallback to encoders");
        telemetry.addData("Front Motors Current", "L:%d  R:%d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("Back Motors Current", "L:%d  R:%d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.addData("Front Motors Target", "L:%d  R:%d",
                frontLeft.getTargetPosition(), frontRight.getTargetPosition());

        // Display AprilTag detection information
        telemetryAprilTag();

        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     * We close the VisionPortal here to release camera resources and save CPU.
     */
    @Override
    public void stop() {
        // Release camera resources when the OpMode stops
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    /**
     * Drives the robot straight forward or backward using all 4 mecanum motors.
     * For mecanum drive, all 4 motors move in the same direction for straight driving.
     *
     * @param speed From 0-1
     * @param distance In specified unit (positive = forward, negative = backward)
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds, OR if the operation times out. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        final double toleranceTicks = TOLERANCE_MM * TICKS_PER_MM;

        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // For mecanum drive, all 4 motors go to the same position for straight driving
        frontLeft.setTargetPosition((int) targetPosition);
        frontRight.setTargetPosition((int) targetPosition);
        backLeft.setTargetPosition((int) targetPosition);
        backRight.setTargetPosition((int) targetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        /*
         * Check if ALL motors are within tolerance of their target position.
         * Previously we only checked front left, which could cause issues if one motor
         * was lagging or stuck. Now we verify all 4 motors have reached their targets.
         */
        boolean allMotorsAtTarget =
                Math.abs(targetPosition - frontLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(targetPosition - frontRight.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(targetPosition - backLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(targetPosition - backRight.getCurrentPosition()) <= toleranceTicks;

        if (!allMotorsAtTarget) {
            driveTimer.reset();
        }

        /*
         * SAFETY: If we've been trying to reach the target for too long, give up and continue.
         * This prevents the autonomous from hanging if a motor fails or gets stuck.
         */
        if (operationTimer.seconds() > DRIVE_TIMEOUT_SECONDS) {
            telemetry.addData("WARNING", "Drive timeout - continuing anyway");
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Strafes the robot laterally (sideways) using mecanum drive.
     * This is the key advantage of mecanum wheels - the ability to move sideways without rotating.
     *
     * Mecanum strafe pattern:
     * - Strafe RIGHT: FL forward, FR backward, BL backward, BR forward
     * - Strafe LEFT:  FL backward, FR forward, BL forward, BR backward
     *
     * @param speed From 0-1
     * @param distance In specified unit (positive = right, negative = left)
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds, OR if the operation times out. "false" otherwise.
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        final double toleranceTicks = TOLERANCE_MM * TICKS_PER_MM;

        /*
         * Mecanum strafe requires a correction factor because lateral movement is less
         * efficient than forward movement due to roller geometry. Typical values are 1.1-1.4.
         * This should be tuned on your specific robot.
         */
        final double STRAFE_CORRECTION = 1.2;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM * STRAFE_CORRECTION);

        /*
         * Mecanum strafe pattern:
         * - To strafe RIGHT: FL +, FR -, BL -, BR +
         * - To strafe LEFT:  FL -, FR +, BL +, BR -
         *
         * With positive distance = right, we use: FL+, FR-, BL-, BR+
         */
        frontLeft.setTargetPosition((int) targetPosition);
        frontRight.setTargetPosition((int) -targetPosition);
        backLeft.setTargetPosition((int) -targetPosition);
        backRight.setTargetPosition((int) targetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Check if ALL motors are within tolerance of their target position
        boolean allMotorsAtTarget =
                Math.abs(targetPosition - frontLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(-targetPosition - frontRight.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(-targetPosition - backLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(targetPosition - backRight.getCurrentPosition()) <= toleranceTicks;

        if (!allMotorsAtTarget) {
            driveTimer.reset();
        }

        // Safety timeout
        if (operationTimer.seconds() > DRIVE_TIMEOUT_SECONDS) {
            telemetry.addData("WARNING", "Strafe timeout - continuing anyway");
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotates the robot in place using all 4 mecanum motors.
     * For rotation, left side motors go opposite direction from right side motors.
     *
     * If the IMU is available and USE_IMU_FOR_ROTATION is true, this method uses closed-loop
     * heading control for accurate rotation. Otherwise, it falls back to encoder-based rotation.
     *
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate (positive = clockwise when viewed from above)
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the robot is within tolerance of the target heading for more than
     *         holdSeconds, OR if the operation times out. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        double angleDeg = angleUnit.toDegrees(angle);

        /*
         * If IMU is available, use closed-loop heading control for more accurate rotation.
         * The IMU measures actual heading independent of wheel slippage.
         */
        if (USE_IMU_FOR_ROTATION && imuAvailable && imu != null) {
            return rotateWithIMU(speed, angleDeg, holdSeconds);
        } else {
            return rotateWithEncoders(speed, angleDeg, holdSeconds);
        }
    }

    /**
     * IMU-based rotation: Uses gyroscope feedback for accurate heading control.
     * This is more accurate than encoder-based rotation because it's not affected by wheel slip.
     */
    private boolean rotateWithIMU(double speed, double angleDeg, double holdSeconds) {
        // Get current heading and calculate target
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

        // On first call for this rotation, set the target heading
        // (We detect "first call" by checking if targetHeadingDeg is still at its reset value)
        if (driveTimer.seconds() < 0.05) {
            targetHeadingDeg = currentHeading + angleDeg;
            // Normalize to -180 to +180
            while (targetHeadingDeg > 180) targetHeadingDeg -= 360;
            while (targetHeadingDeg < -180) targetHeadingDeg += 360;
        }

        double headingError = targetHeadingDeg - currentHeading;
        // Normalize error to -180 to +180
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        // Apply power proportional to error, but clamped to the requested speed
        double turnPower = Math.copySign(Math.min(Math.abs(headingError) / 45.0, 1.0) * speed, headingError);

        // For rotation: left side goes opposite of right side
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(-turnPower);
        backLeft.setPower(-turnPower);
        frontRight.setPower(turnPower);
        backRight.setPower(turnPower);

        // Check if we're within tolerance
        if (Math.abs(headingError) > IMU_HEADING_TOLERANCE_DEG) {
            driveTimer.reset();
        } else {
            // Stop motors when at target
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        // Safety timeout
        if (operationTimer.seconds() > ROTATE_TIMEOUT_SECONDS) {
            telemetry.addData("WARNING", "Rotate timeout - continuing anyway");
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Encoder-based rotation: Fallback when IMU is not available.
     * Uses calculated arc length based on track width to achieve the requested rotation.
     */
    private boolean rotateWithEncoders(double speed, double angleDeg, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        final double toleranceTicks = TOLERANCE_MM * TICKS_PER_MM;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that our wheels
         * need to travel, we just need to multiply the requested angle in radians by the radius
         * of our turning circle.
         */
        double angleRad = Math.toRadians(angleDeg);
        double targetMm = angleRad * (TRACK_WIDTH_MM / 2);

        /*
         * For rotation, left side motors go in the opposite direction from right side motors.
         * This is the same for mecanum as it was for tank drive.
         */
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        // Set target positions - left side negative, right side positive for rotation
        frontLeft.setTargetPosition((int) leftTargetPosition);
        backLeft.setTargetPosition((int) leftTargetPosition);
        frontRight.setTargetPosition((int) rightTargetPosition);
        backRight.setTargetPosition((int) rightTargetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Check if ALL motors are within tolerance
        boolean allMotorsAtTarget =
                Math.abs(leftTargetPosition - frontLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(leftTargetPosition - backLeft.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(rightTargetPosition - frontRight.getCurrentPosition()) <= toleranceTicks &&
                Math.abs(rightTargetPosition - backRight.getCurrentPosition()) <= toleranceTicks;

        if (!allMotorsAtTarget) {
            driveTimer.reset();
        }

        // Safety timeout
        if (operationTimer.seconds() > ROTATE_TIMEOUT_SECONDS) {
            telemetry.addData("WARNING", "Rotate timeout - continuing anyway");
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Helper method to reset encoders on all 4 drive motors.
     * This is called between autonomous state transitions to prepare for the next movement.
     */
    private void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Initialize the IMU for accurate heading control.
     * The IMU is built into the REV Control Hub and provides gyroscope data independent
     * of wheel slippage. If initialization fails, imuAvailable will be false and the
     * robot will fall back to encoder-based rotation.
     */
    private void initIMU() {
        try {
            imu = hardwareMap.get(IMU.class, PickleHardwareNames.IMU_NAME);

            // Define the hub orientation. Adjust these parameters based on how your
            // Control Hub is mounted on the robot.
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw(); // Start with heading = 0
            imuAvailable = true;
            telemetry.addData("IMU", "Initialized successfully");
        } catch (Exception e) {
            imu = null;
            imuAvailable = false;
            telemetry.addData("IMU", "Init failed - using encoder rotation: " + e.getMessage());
        }
    }

    /**
     * Initialize the AprilTag processor and vision portal.
     * This uses the "easy" creation methods which provide sensible defaults.
     * The camera name "Webcam 1" must match the name configured in the robot configuration.
     *
     * If initialization fails (camera not connected, wrong name, etc.), the autonomous
     * will continue using dead-reckoning only.
     */
    private void initAprilTag() {
        try {
            // Create the AprilTag processor using easy defaults.
            // This automatically configures tag family, decimation, and other parameters.
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

            // Create the vision portal using easy defaults with a webcam.
            // The webcam name must match the configuration in the Driver Station.
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, PickleHardwareNames.WEBCAM_NAME), aprilTag);

            telemetry.addData("Camera", "Initialized successfully");
        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
            telemetry.addData("Camera", "Init failed - using dead reckoning: " + e.getMessage());
        }
    }

    /**
     * Add telemetry about AprilTag detections.
     * This displays information about any detected AprilTags including their ID,
     * position (XYZ), orientation (pitch/roll/yaw), and polar coordinates (range/bearing/elevation).
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int expectedGoalId = DecodeField.getGoalAprilTagId(alliance);

        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("Looking for Goal Tag", "ID " + expectedGoalId + " (" + alliance + ")");

        AprilTagDetection goalTag = getGoalDetection();
        if (goalTag != null && goalTag.metadata != null) {
            String matchStatus = (goalTag.id == expectedGoalId) ? " [OUR GOAL]" : " [OTHER GOAL]";
            telemetry.addLine(String.format("GOAL TAG: (ID %d) %s%s", goalTag.id, goalTag.metadata.name, matchStatus));
            telemetry.addLine(String.format("Goal RBE: %6.1f %6.1f %6.1f (inch, deg, deg)",
                    goalTag.ftcPose.range, goalTag.ftcPose.bearing, goalTag.ftcPose.elevation));
        } else {
            telemetry.addLine("GOAL TAG: not visible");
        }

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Tag is in the library - we have full pose information
                String tagType = DecodeField.isObeliskTag(detection.id) ? " [OBELISK-IGNORE]" :
                                 DecodeField.isGoalTag(detection.id) ? " [GOAL]" : "";
                telemetry.addLine(String.format("\n==== (ID %d) %s%s", detection.id, detection.metadata.name, tagType));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                // Tag not in library - only have pixel position
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    /**
     * Returns the best "goal" AprilTag detection to use for navigation, or null if none are visible.
     *
     * <p>Why this exists:</p>
     * <ul>
     *   <li>DECODE fields include tags on the Obelisk (IDs 21, 22, 23) and goals (Red=24, Blue=20)</li>
     *   <li>Obelisk tags should NOT be used for localization to approach the Goal</li>
     * </ul>
     *
     * <p>Selection priority:</p>
     * <ol>
     *   <li>First, look for the current alliance's goal tag (most reliable)</li>
     *   <li>If not found, accept any goal tag (opponent's goal might be visible)</li>
     *   <li>Always reject Obelisk tags using DecodeField.isObeliskTag()</li>
     * </ol>
     *
     * @return The best goal AprilTag detection, or null if none visible
     */
    private AprilTagDetection getGoalDetection() {
        if (aprilTag == null) {
            return null;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection allianceGoalTag = null;
        AprilTagDetection anyGoalTag = null;
        double anyGoalRange = Double.POSITIVE_INFINITY;

        // Get the expected goal tag ID for our alliance
        int expectedTagId = DecodeField.getGoalAprilTagId(alliance);

        for (AprilTagDetection detection : detections) {
            // Skip unknown tags
            if (detection.metadata == null || detection.ftcPose == null) {
                continue;
            }

            // Skip Obelisk tags - they should NOT be used for navigation
            if (DecodeField.isObeliskTag(detection.id)) {
                continue;
            }

            // Check if this is a goal tag
            if (DecodeField.isGoalTag(detection.id)) {
                // Prioritize our alliance's goal tag
                if (detection.id == expectedTagId) {
                    // Found our alliance's goal - use it immediately if it's the first,
                    // or if it's closer than a previous detection
                    if (allianceGoalTag == null || detection.ftcPose.range < allianceGoalTag.ftcPose.range) {
                        allianceGoalTag = detection;
                    }
                } else {
                    // This is the opponent's goal tag - track it as backup
                    if (detection.ftcPose.range < anyGoalRange) {
                        anyGoalTag = detection;
                        anyGoalRange = detection.ftcPose.range;
                    }
                }
            }
        }

        // Prefer our alliance's goal, fall back to any goal
        return allianceGoalTag != null ? allianceGoalTag : anyGoalTag;
    }
}

