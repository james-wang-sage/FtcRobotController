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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pickle.config.PickleHardwareNames;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;

/**
 * Autonomous OpMode: Launch 3 Balls + LEAVE (Rotate-Then-Backward Version)
 *
 * FIELD COORDINATES:
 * - 0° = up (toward far wall)
 * - 90° = right
 * - 180° = down (toward alliance station / near wall)
 * - 270° = left
 *
 * STARTING POSITION:
 * - RED: Inside Launch Zone, facing 45° (toward red goal corner)
 * - BLUE: Inside Launch Zone, facing 315° (toward blue goal corner)
 * - Pre-loaded with 3 balls
 *
 * STRATEGY (5-Step Sequence):
 * 1. Launch 3 balls (same process as TeleOp for consistent power)
 * 2. Wait 1 second
 * 3. Move backward 6 inches (clear the launch zone edge for rotation)
 * 4. ROTATE to face 0° field (straight up toward far wall):
 *    - RED: Turn LEFT 45° (from 45° to 0°)
 *    - BLUE: Turn RIGHT 45° (from 315° to 0°)
 * 5. Move backward 60 inches (toward 180° field, exit zone)
 * 6. Wait until AUTO ends to score LEAVE (3 Ranking Points)
 *
 * WHY THIS SEQUENCE?
 * - Initial 6" backward clears space for safe rotation
 * - Straight backward after rotation is more efficient than diagonal
 * - Uses all 4 motors for maximum traction
 * - IMU provides accurate rotation control
 *
 * ALLIANCE MIRRORING:
 * - RED:  Faces 45° initially, rotate LEFT 45° to face 0°, then backward
 * - BLUE: Faces 315° initially, rotate RIGHT 45° to face 0°, then backward
 *
 * ALLIANCE SELECTION:
 * - During INIT, press gamepad1.A for RED or gamepad1.B for BLUE
 * - Default: RED alliance
 */
@Autonomous(name = "PickleAutoLaunchRotate", group = "Pickle")
public class PickleAutoLaunchRotate extends LinearOpMode {

    // Launcher configuration - MATCHED TO TELEOP (which works for all 3 balls!)
    private static final double LAUNCHER_TARGET_VELOCITY = 1100;  // ticks/second
    private static final double LAUNCHER_MIN_VELOCITY = 1000;     // minimum before launching
    private static final double SPIN_UP_TIMEOUT_SECONDS = 1.0;    // max wait for velocity
    private static final double FEED_TIME_SECONDS = 0.35;         // feeder run time per shot
    private static final double INTERVAL_BETWEEN_SHOTS = 0.30;    // seconds between shots
    private static final int TOTAL_BALLS = 3;                     // number of balls to launch

    // Movement configuration
    private static final double WAIT_AFTER_LAUNCH_SECONDS = 1.0;  // wait after launching (step 2)
    private static final double DRIVE_SPEED = 0.5;                // motor power for backward drive

    // Encoder configuration for distance-based movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7;     // goBILDA 5203-2402-0019 (312 RPM)
    private static final double WHEEL_DIAMETER_MM = 96.0;         // goBILDA 96mm mecanum wheels
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);

    // Step 3: Initial backward movement (before rotation) to clear launch zone edge
    private static final double INITIAL_BACKWARD_MM = 152.4;      // 6 inches

    // Step 5: Main backward movement (after rotation) toward 180° field
    private static final double EXIT_DISTANCE_MM = 1524.0;        // 60 inches

    // Rotation configuration (Step 4)
    // IMU is reset at start, so target heading is relative to initial position
    // RED: Turn LEFT 45° (counterclockwise) → IMU reads +45°
    // BLUE: Turn RIGHT 45° (clockwise) → IMU reads -45°
    private static final double RED_TARGET_HEADING = 45.0;        // Turn LEFT to face 0° field
    private static final double BLUE_TARGET_HEADING = -45.0;      // Turn RIGHT to face 0° field
    private static final double HEADING_TOLERANCE = 2.0;          // degrees tolerance for rotation
    private static final double ROTATION_SPEED = 0.4;             // motor power for rotation
    private static final double ROTATION_KP = 0.02;               // proportional gain for smooth stop

    // Alliance selection
    private Alliance alliance = Alliance.RED;

    // Hardware - Launcher
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Hardware - Drive motors (mecanum)
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Hardware - IMU for rotation
    private IMU imu = null;

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
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);

        // Set motor directions (right side reversed for mecanum)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU (critical for rotation control)
        try {
            imu = hardwareMap.get(IMU.class, PickleHardwareNames.IMU_NAME);
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();  // Reset to 0° at start
        } catch (Exception e) {
            imu = null;
            telemetry.addData("IMU ERROR", e.getMessage());
        }

        telemetry.addData("Status", "Ready! Select alliance...");
        telemetry.addData("IMU", imu != null ? "Ready" : "NOT AVAILABLE - rotation disabled");
        telemetry.update();

        // =====================================
        // ALLIANCE SELECTION (during init)
        // =====================================
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                alliance = Alliance.RED;
            } else if (gamepad1.b) {
                alliance = Alliance.BLUE;
            }

            telemetry.addData("=== ALLIANCE SELECTION ===", "");
            telemetry.addData("Current Alliance", alliance == Alliance.RED ? "RED (A)" : "BLUE (B)");
            telemetry.addData("Press A", "Select RED alliance");
            telemetry.addData("Press B", "Select BLUE alliance");
            telemetry.addData("", "");
            telemetry.addData("Strategy", "Launch %d balls → wait %.1fs",
                              TOTAL_BALLS, WAIT_AFTER_LAUNCH_SECONDS);
            telemetry.addData("Then", "Back %.0fmm → Rotate %s %.0f° → Back %.0fmm",
                              INITIAL_BACKWARD_MM,
                              alliance == Alliance.RED ? "LEFT" : "RIGHT",
                              Math.abs(alliance == Alliance.RED ? RED_TARGET_HEADING : BLUE_TARGET_HEADING),
                              EXIT_DISTANCE_MM);
            telemetry.addData("IMU", imu != null ? "Ready" : "NOT AVAILABLE");
            telemetry.update();

            sleep(50);
        }

        if (!opModeIsActive()) return;

        // Reset IMU yaw at match start for accurate rotation
        if (imu != null) {
            imu.resetYaw();
        }

        // =====================================
        // MAIN AUTONOMOUS SEQUENCE
        // =====================================
        telemetry.addData("Status", "Starting launch sequence");
        telemetry.update();

        // Start spinning up the launcher
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Wait for launcher to reach minimum velocity
        telemetry.addData("Status", "Spinning up launcher...");
        telemetry.update();
        timer.reset();
        while (opModeIsActive() &&
               launcher.getVelocity() < LAUNCHER_MIN_VELOCITY &&
               timer.seconds() < SPIN_UP_TIMEOUT_SECONDS) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Status", "Initial SPIN_UP");
            telemetry.addData("Launcher Speed", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
            telemetry.update();
            sleep(10);
        }

        // Launch each ball (same as PickleAutoLaunch - matching TeleOp behavior)
        for (int ball = 1; ball <= TOTAL_BALLS && opModeIsActive(); ball++) {

            // SPIN_UP STATE
            timer.reset();
            while (opModeIsActive() &&
                   launcher.getVelocity() < LAUNCHER_MIN_VELOCITY &&
                   timer.seconds() < SPIN_UP_TIMEOUT_SECONDS) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Status", "Ball %d: SPIN_UP", ball);
                telemetry.addData("Launcher Speed", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
                telemetry.update();
                sleep(10);
            }

            // LAUNCH STATE - re-check velocity right before feeding
            // Reset timer for this check (previous spin-up may have consumed the timeout)
            timer.reset();
            while (opModeIsActive() && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Status", "Ball %d: LAUNCH re-check (velocity dropped)", ball);
                telemetry.addData("Launcher Speed", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
                telemetry.update();
                sleep(10);
                if (timer.seconds() > SPIN_UP_TIMEOUT_SECONDS) break;  // Timeout for re-check
            }

            double velocityBeforeFeed = launcher.getVelocity();

            // LAUNCHING STATE - feed the ball
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(1.0);
            timer.reset();
            while (opModeIsActive() && timer.seconds() < FEED_TIME_SECONDS) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Status", "Ball %d: LAUNCHING", ball);
                telemetry.addData("Launcher Speed", "%.0f", launcher.getVelocity());
                telemetry.update();
                sleep(10);
            }

            // Stop feeders
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            telemetry.addData("Status", "Ball %d launched at %.0f!", ball, velocityBeforeFeed);
            telemetry.update();

            // WAIT_BETWEEN STATE
            if (ball < TOTAL_BALLS) {
                timer.reset();
                while (opModeIsActive() && timer.seconds() < INTERVAL_BETWEEN_SHOTS) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    telemetry.addData("Status", "WAIT_BETWEEN");
                    telemetry.update();
                    sleep(10);
                }
            }
        }

        // =====================================
        // STOP LAUNCHER (Step 1 Complete)
        // =====================================
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // =====================================
        // STEP 2: WAIT 1 SECOND
        // =====================================
        telemetry.addData("Status", "Launch complete! Waiting %.1fs...", WAIT_AFTER_LAUNCH_SECONDS);
        telemetry.update();

        sleep((long) (WAIT_AFTER_LAUNCH_SECONDS * 1000));

        if (!opModeIsActive()) return;

        // =====================================
        // STEP 3: MOVE BACKWARD 6 INCHES (before rotation)
        // This clears space for rotation
        // =====================================
        telemetry.addData("Status", "Step 3: Moving backward 6 inches...");
        telemetry.update();

        driveBackward(INITIAL_BACKWARD_MM, 2.0);  // 6 inches with 2 second timeout

        if (!opModeIsActive()) return;

        // =====================================
        // STEP 4: ROTATE TO FACE 0° FIELD DIRECTION
        // =====================================
        if (imu != null) {
            double targetHeading = (alliance == Alliance.RED) ? RED_TARGET_HEADING : BLUE_TARGET_HEADING;

            telemetry.addData("Status", "Step 4: Rotating to face 0° field...");
            telemetry.addData("Target IMU Heading", "%.1f°", targetHeading);
            telemetry.update();

            timer.reset();
            double timeoutSeconds = 3.0;

            while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double headingError = targetHeading - currentHeading;

                // Normalize error to -180 to 180
                while (headingError > 180) headingError -= 360;
                while (headingError < -180) headingError += 360;

                telemetry.addData("Status", "Rotating...");
                telemetry.addData("Current Heading", "%.1f°", currentHeading);
                telemetry.addData("Target Heading", "%.1f°", targetHeading);
                telemetry.addData("Error", "%.1f°", headingError);
                telemetry.update();

                // Check if we're within tolerance
                if (Math.abs(headingError) < HEADING_TOLERANCE) {
                    break;
                }

                // Calculate rotation power with proportional control for smooth stopping
                double rotationPower = headingError * ROTATION_KP;

                // Clamp power to rotation speed limits
                if (rotationPower > ROTATION_SPEED) rotationPower = ROTATION_SPEED;
                if (rotationPower < -ROTATION_SPEED) rotationPower = -ROTATION_SPEED;

                // Ensure minimum power to overcome friction
                if (Math.abs(rotationPower) < 0.15 && Math.abs(headingError) > HEADING_TOLERANCE) {
                    rotationPower = (headingError > 0) ? 0.15 : -0.15;
                }

                // Apply rotation: positive = turn left (counterclockwise)
                // To turn left: right motors forward, left motors backward
                frontLeft.setPower(-rotationPower);
                backLeft.setPower(-rotationPower);
                frontRight.setPower(rotationPower);
                backRight.setPower(rotationPower);

                sleep(10);
            }

            // Stop rotation
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Brief pause to stabilize
            sleep(200);

            double finalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Status", "Rotation complete!");
            telemetry.addData("Final Heading", "%.1f° (target: %.1f°)", finalHeading, targetHeading);
            telemetry.update();
            sleep(200);
        } else {
            // IMU FAILURE FALLBACK:
            // Without IMU, we can't rotate accurately. The robot is still facing 45°/315°.
            // If we drive backward now, it will go toward ~225° (RED) or ~135° (BLUE), not 180°.
            // Option: Skip the long backward drive and just stay put (safer than wrong direction)
            telemetry.addData("WARNING", "No IMU - cannot rotate!");
            telemetry.addData("FALLBACK", "Skipping main backward drive (wrong direction without rotation)");
            telemetry.update();
            sleep(1000);

            // Skip to waiting for AUTO end - robot has already moved 6" back from step 3
            // This is safer than driving 60" in the wrong direction
            telemetry.addData("Status", "IMU FAILED - Waiting for AUTO to end...");
            telemetry.update();
            while (opModeIsActive()) {
                telemetry.addData("Status", "IMU FAILED - Robot stationary");
                telemetry.addData("Note", "Only moved 6\" back, may not score LEAVE");
                telemetry.update();
                sleep(100);
            }
            return;  // Exit runOpMode early
        }

        if (!opModeIsActive()) return;

        // =====================================
        // STEP 5: MOVE BACKWARD 60 INCHES (main exit)
        // Robot now faces 0° field, so backward = 180° field direction
        // =====================================
        telemetry.addData("Status", "Step 5: Moving backward 60 inches to exit zone...");
        telemetry.update();

        driveBackward(EXIT_DISTANCE_MM, 6.0);  // 60 inches with 6 second timeout

        // =====================================
        // STEP 6: WAIT UNTIL AUTO ENDS
        // Robot is in position, just wait to score LEAVE
        // =====================================
        telemetry.addData("Status", "IN POSITION - Waiting for AUTO to end...");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Result", "Launched %d balls", TOTAL_BALLS);
        telemetry.addData("Total Distance", "%.0f + %.0f = %.0f mm",
                          INITIAL_BACKWARD_MM, EXIT_DISTANCE_MM, INITIAL_BACKWARD_MM + EXIT_DISTANCE_MM);
        if (imu != null) {
            telemetry.addData("Final Heading", "%.1f°",
                              imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        telemetry.update();

        // Wait until the autonomous period ends (opMode becomes inactive)
        while (opModeIsActive()) {
            telemetry.addData("Status", "WAITING FOR AUTO END - LEAVE will score!");
            telemetry.addData("Alliance", alliance);
            if (imu != null) {
                telemetry.addData("Current Heading", "%.1f°",
                                  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Helper method to drive backward a specified distance using encoder-based movement.
     * Uses all 4 motors for straight backward movement.
     *
     * @param distanceMM Distance to travel in millimeters
     * @param timeoutSeconds Maximum time to allow for the movement
     */
    private void driveBackward(double distanceMM, double timeoutSeconds) {
        // Calculate target encoder counts
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions (negative = backward)
        int backwardTarget = -targetCounts;
        frontLeft.setTargetPosition(backwardTarget);
        frontRight.setTargetPosition(backwardTarget);
        backLeft.setTargetPosition(backwardTarget);
        backRight.setTargetPosition(backwardTarget);

        // Switch to RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving (power must be positive for RUN_TO_POSITION)
        frontLeft.setPower(DRIVE_SPEED);
        frontRight.setPower(DRIVE_SPEED);
        backLeft.setPower(DRIVE_SPEED);
        backRight.setPower(DRIVE_SPEED);

        // Wait until all motors reach target position
        timer.reset();
        while (opModeIsActive() &&
               (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
               timer.seconds() < timeoutSeconds) {

            double progress = Math.abs(frontLeft.getCurrentPosition()) / COUNTS_PER_MM;
            telemetry.addData("Driving Backward", "%.0f / %.0f mm", progress, distanceMM);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), timeoutSeconds);
            telemetry.update();
            sleep(50);
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // CRITICAL: Switch back to RUN_USING_ENCODER mode after RUN_TO_POSITION
        // This allows subsequent rotation commands (setPower) to work correctly
        // Without this, the PID controller from RUN_TO_POSITION fights rotation!
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brief pause to stabilize
        sleep(100);
    }
}
