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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pickle.config.PickleHardwareNames;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;

/**
 * Simple Autonomous OpMode: LEAVE Only (No Launching)
 *
 * USE THIS WHEN:
 * - Alliance partner takes the close starting position (under the goal zone)
 * - We start at the FAR position (away from goal)
 * - We cannot launch balls (partner in the way or not in launch zone)
 * - We just need to move forward to score LEAVE points
 *
 * STRATEGY:
 * 1. Move FORWARD 36 inches (toward the field center)
 * 2. Stop and wait until AUTO period ends
 * 3. Score LEAVE (3 Ranking Points)
 *
 * WORKS FOR BOTH ALLIANCES:
 * - RED and BLUE both just move forward 36 inches
 * - No rotation needed, no launching
 * - Simple and reliable
 *
 * ALLIANCE SELECTION:
 * - During INIT, press gamepad1.A for RED or gamepad1.B for BLUE
 * - Default: RED alliance
 * - (Alliance selection is kept for consistency with other OpModes,
 *    but movement is the same for both)
 */
@Autonomous(name = "PickleAutoLaunchFar", group = "Pickle")
public class PickleAutoLaunchFar extends LinearOpMode {

    // Movement configuration
    private static final double DRIVE_SPEED = 0.5;                // motor power for forward drive
    private static final double FORWARD_DISTANCE_MM = 914.4;      // 36 inches in mm

    // Encoder configuration for distance-based movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7;     // goBILDA 5203-2402-0019 (312 RPM)
    private static final double WHEEL_DIAMETER_MM = 96.0;         // goBILDA 96mm mecanum wheels
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);

    // Alliance selection (for display purposes - movement is same for both)
    private Alliance alliance = Alliance.RED;

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

        telemetry.addData("Status", "Ready! Select alliance...");
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

            telemetry.addData("=== FAR POSITION AUTO ===", "");
            telemetry.addData("Current Alliance", alliance == Alliance.RED ? "RED (A)" : "BLUE (B)");
            telemetry.addData("Press A", "Select RED alliance");
            telemetry.addData("Press B", "Select BLUE alliance");
            telemetry.addData("", "");
            telemetry.addData("Strategy", "Move FORWARD 36 inches → Wait for AUTO end");
            telemetry.addData("Purpose", "LEAVE points only (no launching)");
            telemetry.addData("Use when", "Partner takes close position under goal");
            telemetry.update();

            sleep(50);
        }

        if (!opModeIsActive()) return;

        // =====================================
        // STEP 1: MOVE FORWARD 36 INCHES
        // =====================================
        telemetry.addData("Status", "Moving forward 36 inches...");
        telemetry.addData("Alliance", alliance);
        telemetry.update();

        driveForward(FORWARD_DISTANCE_MM, 5.0);  // 36 inches with 5 second timeout

        if (!opModeIsActive()) return;

        // =====================================
        // STEP 2: WAIT UNTIL AUTO ENDS
        // =====================================
        telemetry.addData("Status", "IN POSITION - Waiting for AUTO to end...");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Distance Moved", "36 inches (%.0f mm)", FORWARD_DISTANCE_MM);
        telemetry.addData("Result", "LEAVE will score at AUTO end");
        telemetry.update();

        // Wait until the autonomous period ends
        while (opModeIsActive()) {
            telemetry.addData("Status", "WAITING FOR AUTO END - LEAVE will score!");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Note", "No balls launched (far position)");
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Helper method to drive forward a specified distance using encoder-based movement.
     * Uses all 4 motors for straight forward movement.
     *
     * @param distanceMM Distance to travel in millimeters
     * @param timeoutSeconds Maximum time to allow for the movement
     */
    private void driveForward(double distanceMM, double timeoutSeconds) {
        // Calculate target encoder counts
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions (positive = forward)
        frontLeft.setTargetPosition(targetCounts);
        frontRight.setTargetPosition(targetCounts);
        backLeft.setTargetPosition(targetCounts);
        backRight.setTargetPosition(targetCounts);

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
            telemetry.addData("Driving Forward", "%.0f / %.0f mm", progress, distanceMM);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), timeoutSeconds);
            telemetry.update();
            sleep(50);
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Switch back to RUN_USING_ENCODER mode (good practice)
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brief pause to stabilize
        sleep(100);
    }
}
