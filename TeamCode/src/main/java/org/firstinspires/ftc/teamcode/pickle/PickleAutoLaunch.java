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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pickle.config.PickleHardwareNames;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;

/**
 * Simple Autonomous OpMode: Launch 3 Balls + LEAVE
 *
 * FIELD COORDINATES:
 * - 0° = up (toward far wall)
 * - 90° = right
 * - 180° = down (toward alliance station / near wall)
 * - 270° = left
 *
 * STARTING POSITION:
 * - Red alliance, inside the Launch Zone (white triangle, top-right corner)
 * - Robot FACING 45° (toward the goal in the corner)
 * - Pre-loaded with 3 balls
 *
 * STRATEGY:
 * 1. Spin up launcher and launch 3 balls (while stationary, facing 45° at goal)
 * 2. Wait 3 seconds (optional delay for ball settling / alliance coordination)
 * 3. Move toward 180° (down on field) to exit the Launch Zone
 *    - Robot faces 45°, robot's right is 135° field
 *    - 180° is between right (135°) and back (225°) = backward-right diagonal
 *    - Uses mecanum diagonal drive pattern (FR and BL only)
 * 4. Score LEAVE (3 Ranking Points) by being fully outside the Launch Zone at AUTO end
 *
 * WHY DIAGONAL STRAFE?
 * - Robot faces toward goal to launch (45° for Red, 135° for Blue)
 * - Exit direction is 180° (straight down on field) for both alliances
 * - Mecanum wheels enable movement in any direction without rotating
 * - Robot maintains goal alignment throughout AUTO
 *
 * ALLIANCE MIRRORING:
 * - RED:  Faces 45°,  exit 180° = back-right diagonal (FR+BL pair, negative)
 * - BLUE: Faces 315°, exit 180° = back-left diagonal  (FL+BR pair, negative)
 *
 * ALLIANCE SELECTION (same as TeleOp):
 * - During INIT, press gamepad1.A for RED or gamepad1.B for BLUE
 * - Default: RED alliance
 *
 * This autonomous scores artifacts AND earns Movement RP contribution (LEAVE).
 * No IMU or vision required - simple encoder-based diagonal movement.
 */
@Autonomous(name = "PickleAutoLaunch", group = "Pickle")
public class PickleAutoLaunch extends LinearOpMode {

    // Launcher configuration - MATCHED TO TELEOP (which works for all 3 balls!)
    private static final double LAUNCHER_TARGET_VELOCITY = 1100;  // ticks/second (same as TeleOp)
    private static final double LAUNCHER_MIN_VELOCITY = 1000;     // minimum before launching (same as TeleOp)
    private static final double SPIN_UP_TIMEOUT_SECONDS = 1.0;    // max wait for velocity (same as TeleOp) - prevents hang!
    private static final double FEED_TIME_SECONDS = 0.35;         // feeder run time per shot (same as TeleOp: 0.35)
    private static final double INTERVAL_BETWEEN_SHOTS = 0.30;    // seconds between shots (same as TeleOp: 0.30)
    private static final int TOTAL_BALLS = 3;                     // number of balls to launch

    // LEAVE configuration - move out of Launch Zone using encoders
    private static final double WAIT_BEFORE_DRIVE_SECONDS = 3.0;  // wait after launching before moving
    private static final double DRIVE_SPEED = 0.5;                // motor power (0.0 to 1.0)

    // Encoder configuration for distance-based movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7;     // goBILDA 5203-2402-0019 (312 RPM)
    private static final double WHEEL_DIAMETER_MM = 96.0;         // goBILDA 96mm mecanum wheels
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);

    // Distance to exit launch zone (approximately 1.5 tiles = 36 inches = 914mm)
    // Increased to ensure we're fully outside the zone with margin
    private static final double EXIT_DISTANCE_MM = 1500.0;        // ~59 inches (+6" from 1350mm)

    // For diagonal movement, we need a correction factor
    // Diagonal uses only 2 motors, so effective movement is ~70% of straight drive
    private static final double DIAGONAL_CORRECTION = 1.4;        // compensate for diagonal inefficiency

    // Angle correction for each alliance:
    // RED:  Pure FR+BL diagonal drifts ~35° left (215° instead of 180°)
    //       Add FL+BR forward to steer right → back toward 180°
    // BLUE: Pure FL+BR diagonal drifts ~10° left (190° instead of 180°)
    //       Add FR+BL backward to steer right → back toward 180°
    private static final double RED_ANGLE_CORRECTION = 0.35;      // FL+BR forward power for RED
    private static final double BLUE_ANGLE_CORRECTION = 0.15;     // FR+BL backward power for BLUE

    // Alliance selection (determines diagonal direction)
    // RED:  back-right diagonal (FR+BL pair, backward)
    // BLUE: back-left diagonal (FL+BR pair, backward)
    private Alliance alliance = Alliance.RED;  // Default to RED

    // Hardware - Launcher
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Hardware - Drive motors (mecanum)
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // =====================================
        // INITIALIZATION
        // =====================================
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize launcher motor
        launcher = hardwareMap.get(DcMotorEx.class, PickleHardwareNames.LAUNCHER_MOTOR);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(50, 0, 0, 14));

        // Initialize feeder servos
        leftFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.LEFT_FEEDER_SERVO);
        rightFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.RIGHT_FEEDER_SERVO);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop feeders initially
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Initialize drive motors (same configuration as TeleOp)
        frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);

        // Set motor directions (right side reversed for mecanum)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to BRAKE for controlled stops
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders for accurate distance tracking
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_USING_ENCODER for now (will switch to RUN_TO_POSITION for movement)
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready! Select alliance...");
        telemetry.addData("Strategy", "Launch %d balls, wait %.1fs, move %.0fmm",
                          TOTAL_BALLS, WAIT_BEFORE_DRIVE_SECONDS, EXIT_DISTANCE_MM);
        telemetry.update();

        // =====================================
        // ALLIANCE SELECTION (during init) - same buttons as TeleOp
        // =====================================
        // Press A for RED, B for BLUE while waiting for start
        while (!isStarted() && !isStopRequested()) {
            // Alliance selection with gamepad (same as TeleOp)
            if (gamepad1.a) {
                alliance = Alliance.RED;
            } else if (gamepad1.b) {
                alliance = Alliance.BLUE;
            }

            // Display current selection
            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Current Alliance", alliance == Alliance.RED ? "RED (A)" : "BLUE (B)");
            telemetry.addData("Press A", "Select RED alliance");
            telemetry.addData("Press B", "Select BLUE alliance");
            telemetry.addData("", "");
            telemetry.addData("Strategy", "Launch %d balls → wait %.1fs → move %.0fmm",
                              TOTAL_BALLS, WAIT_BEFORE_DRIVE_SECONDS, EXIT_DISTANCE_MM);
            telemetry.addData("Diagonal", alliance == Alliance.RED ?
                              "Back-RIGHT (FR+BL)" : "Back-LEFT (FL+BR)");
            telemetry.addData("Angle Correction", "%.0f%% (steer toward 180° field)",
                              (alliance == Alliance.RED ? RED_ANGLE_CORRECTION : BLUE_ANGLE_CORRECTION) * 100);
            telemetry.update();

            sleep(50);
        }

        if (!opModeIsActive()) return;

        // =====================================
        // MAIN AUTONOMOUS SEQUENCE
        // =====================================
        telemetry.addData("Status", "Starting launch sequence");
        telemetry.update();

        // Start spinning up the launcher immediately
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Wait for launcher to reach minimum velocity (with timeout to prevent hang)
        // CRITICAL: Continuously command velocity like TeleOp does
        telemetry.addData("Status", "Spinning up launcher...");
        telemetry.update();
        timer.reset();
        while (opModeIsActive() &&
               launcher.getVelocity() < LAUNCHER_MIN_VELOCITY &&
               timer.seconds() < SPIN_UP_TIMEOUT_SECONDS) {
            // Keep commanding velocity every iteration (like TeleOp)
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

            telemetry.addData("Status", "Initial SPIN_UP");
            telemetry.addData("Launcher Speed", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Spin-up Time", "%.2f / %.1f sec", timer.seconds(), SPIN_UP_TIMEOUT_SECONDS);
            telemetry.update();
            sleep(10);  // 10ms loop like TeleOp's ~20Hz rate
        }

        // Log if we timed out (fire anyway - better than hanging)
        if (launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
            telemetry.addData("WARNING", "Spin-up timed out! Firing anyway at %.0f", launcher.getVelocity());
            telemetry.update();
        }

        // Launch each ball with interval - EXACTLY LIKE TELEOP
        // Key: Continuously command launcher.setVelocity() in EVERY loop iteration
        // TeleOp does this in every state, which keeps the PIDF controller actively responding
        for (int ball = 1; ball <= TOTAL_BALLS && opModeIsActive(); ball++) {

            // === SPIN_UP STATE (from TeleOp) ===
            // Wait for velocity with timeout, continuously commanding target velocity
            timer.reset();
            while (opModeIsActive() &&
                   launcher.getVelocity() < LAUNCHER_MIN_VELOCITY &&
                   timer.seconds() < SPIN_UP_TIMEOUT_SECONDS) {
                // CRITICAL: Continuously command velocity (like TeleOp does)
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                telemetry.addData("Status", "Ball %d: SPIN_UP", ball);
                telemetry.addData("Launcher Speed", "%.0f / %.0f",
                                  launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
                telemetry.addData("Spin-up Time", "%.2f / %.1f sec",
                                  timer.seconds(), SPIN_UP_TIMEOUT_SECONDS);
                telemetry.update();
                sleep(10);
            }

            // === LAUNCH STATE (from TeleOp) ===
            // Re-check velocity RIGHT BEFORE feeding (like TeleOp does)
            // If velocity dropped, go back to SPIN_UP
            while (opModeIsActive() && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                // Velocity dropped - wait for recovery (with timeout)
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Status", "Ball %d: LAUNCH re-check (velocity dropped!)", ball);
                telemetry.addData("Launcher Speed", "%.0f / %.0f (recovering)",
                                  launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
                telemetry.update();
                sleep(10);

                // Break out if we've waited too long (fire anyway)
                if (timer.seconds() > SPIN_UP_TIMEOUT_SECONDS * 2) {
                    telemetry.addData("WARNING", "Re-check timeout, firing anyway");
                    break;
                }
            }

            // Log pre-shot velocity
            double velocityBeforeFeed = launcher.getVelocity();
            telemetry.addData("Status", "Ball %d: LAUNCH at %.0f", ball, velocityBeforeFeed);
            telemetry.update();

            // === LAUNCHING STATE (from TeleOp) ===
            // Feed the ball while continuously commanding velocity
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(1.0);
            timer.reset();
            while (opModeIsActive() && timer.seconds() < FEED_TIME_SECONDS) {
                // CRITICAL: Keep commanding velocity during feed (like TeleOp does)
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                telemetry.addData("Status", "Ball %d: LAUNCHING", ball);
                telemetry.addData("Launcher Speed", "%.0f (feeding)", launcher.getVelocity());
                telemetry.update();
                sleep(10);
            }

            // Stop feeders
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            double velocityAfterFeed = launcher.getVelocity();
            telemetry.addData("Status", "Ball %d launched!", ball);
            telemetry.addData("Velocity Drop", "%.0f → %.0f (Δ%.0f)",
                              velocityBeforeFeed, velocityAfterFeed, velocityAfterFeed - velocityBeforeFeed);
            telemetry.update();

            // === WAIT_BETWEEN STATE (from TeleOp) ===
            // Wait interval before next shot, continuously commanding velocity
            if (ball < TOTAL_BALLS) {
                timer.reset();
                while (opModeIsActive() && timer.seconds() < INTERVAL_BETWEEN_SHOTS) {
                    // CRITICAL: Keep commanding velocity during wait (like TeleOp does)
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                    telemetry.addData("Status", "WAIT_BETWEEN: %.2f / %.2f sec",
                                      timer.seconds(), INTERVAL_BETWEEN_SHOTS);
                    telemetry.addData("Velocity Recovery", "%.0f / %.0f",
                                      launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
                    telemetry.update();
                    sleep(10);
                }
            }
        }

        // =====================================
        // STOP LAUNCHER
        // =====================================
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Status", "Launch complete! Waiting %.1fs before driving...", WAIT_BEFORE_DRIVE_SECONDS);
        telemetry.update();

        // =====================================
        // WAIT BEFORE DRIVING
        // =====================================
        sleep((long) (WAIT_BEFORE_DRIVE_SECONDS * 1000));

        if (!opModeIsActive()) return;

        // =====================================
        // MOVE OUT OF LAUNCH ZONE (LEAVE) - ENCODER BASED
        // =====================================
        telemetry.addData("Status", "Moving toward 180° (down) to LEAVE zone...");
        telemetry.addData("Alliance", alliance);
        telemetry.update();

        // Alliance-specific diagonal movement:
        //
        // RED ALLIANCE (robot faces 45° toward goal):
        //   - Target 180° is back-right from robot's view
        //   - Use FR+BL pair (both backward/negative)
        //
        // BLUE ALLIANCE (robot faces 315° toward goal):
        //   - Target 180° is back-left from robot's view
        //   - Use FL+BR pair (both backward/negative)

        // Calculate target encoder counts for the diagonal movement
        // Apply diagonal correction factor since only 2 motors are used
        int targetCounts = (int)(EXIT_DISTANCE_MM * COUNTS_PER_MM * DIAGONAL_CORRECTION);

        // Variables for the active motor pair (depends on alliance)
        DcMotor motor1, motor2;
        int targetPosition;

        if (alliance == Alliance.RED) {
            // RED: Back-right diagonal (FR+BL pair, negative direction)
            // Robot faces 45°, exit at 180° = 135° robot-relative (back-right)
            motor1 = frontRight;
            motor2 = backLeft;
            targetPosition = -targetCounts;  // Backward movement
        } else {
            // BLUE: Back-left diagonal (FL+BR pair, negative direction)
            // Robot faces 315° toward goal, exit at 180° = back-left from robot's view
            motor1 = frontLeft;
            motor2 = backRight;
            targetPosition = -targetCounts;  // Backward movement (was incorrectly positive)
        }

        // Reset encoders before movement
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        motor1.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition);

        // Switch to RUN_TO_POSITION mode
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start movement (power must be positive for RUN_TO_POSITION)
        // Main diagonal pair (FR+BL for RED, FL+BR for BLUE) gets full power
        // Add correction power to the OTHER pair to steer toward 180° field direction
        //
        // For RED: Pure FR+BL diagonal drifts to ~215° (35° left of target 180°)
        //          Add FL+BR forward power to add rightward strafe, steering back to 180°
        //
        // For BLUE: May need opposite correction (adjust ANGLE_CORRECTION_POWER sign if needed)
        if (alliance == Alliance.RED) {
            // RED: Main diagonal FR+BL backward (via RUN_TO_POSITION negative targets)
            // Drifts to ~215° (35° left of 180°), add FL+BR forward to steer right
            frontLeft.setPower(DRIVE_SPEED * RED_ANGLE_CORRECTION);    // correction (forward)
            frontRight.setPower(DRIVE_SPEED);                          // main diagonal
            backLeft.setPower(DRIVE_SPEED);                            // main diagonal
            backRight.setPower(DRIVE_SPEED * RED_ANGLE_CORRECTION);    // correction (forward)
        } else {
            // BLUE: Main diagonal FL+BR backward (via RUN_TO_POSITION negative targets)
            // Drifts to ~190° (10° left of 180°), add FR+BL BACKWARD to steer right
            // IMPORTANT: FR+BL are in RUN_USING_ENCODER mode, so:
            //   - Positive power = forward (wrong!)
            //   - Negative power = backward (correct for rightward steering)
            frontLeft.setPower(DRIVE_SPEED);                           // main diagonal
            frontRight.setPower(-DRIVE_SPEED * BLUE_ANGLE_CORRECTION); // correction (BACKWARD!)
            backLeft.setPower(-DRIVE_SPEED * BLUE_ANGLE_CORRECTION);   // correction (BACKWARD!)
            backRight.setPower(DRIVE_SPEED);                           // main diagonal
        }

        // Wait until motors reach target position (with timeout for safety)
        timer.reset();
        double timeoutSeconds = 5.0;  // Safety timeout
        while (opModeIsActive() &&
               (motor1.isBusy() || motor2.isBusy()) &&
               timer.seconds() < timeoutSeconds) {

            telemetry.addData("Status", "Moving diagonal to exit zone...");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Motor1 Position", "%d / %d", motor1.getCurrentPosition(), targetPosition);
            telemetry.addData("Motor2 Position", "%d / %d", motor2.getCurrentPosition(), targetPosition);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), timeoutSeconds);
            telemetry.update();
            sleep(50);
        }

        // =====================================
        // STOP ALL MOTORS
        // =====================================
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Status", "AUTO COMPLETE!");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Result", "Launched %d balls + LEAVE scored", TOTAL_BALLS);
        telemetry.addData("Distance Moved", "%.0f mm (target: %.0f mm)",
                          Math.abs(motor1.getCurrentPosition()) / COUNTS_PER_MM / DIAGONAL_CORRECTION,
                          EXIT_DISTANCE_MM);
        telemetry.update();

        // Keep telemetry visible briefly before OpMode ends
        sleep(500);
    }
}
