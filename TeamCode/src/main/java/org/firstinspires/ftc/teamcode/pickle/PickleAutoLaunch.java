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

/**
 * Simple Autonomous OpMode: Launch 3 Balls
 *
 * STARTING POSITION:
 * - Red alliance, next to goal zone
 * - Robot facing perpendicular to the goal zone border
 * - Pre-loaded with 3 balls
 *
 * STRATEGY:
 * - Robot stays stationary (no driving)
 * - Spin up launcher
 * - Launch 3 balls in sequence with 1-second intervals
 *
 * This is a minimal autonomous for quickly scoring pre-loaded balls
 * without any complex path following or vision processing.
 */
@Autonomous(name = "PickleAutoLaunch", group = "Pickle")
public class PickleAutoLaunch extends LinearOpMode {

    // Launcher configuration (same as TeleOp)
    private static final double LAUNCHER_TARGET_VELOCITY = 1100;  // ticks/second
    private static final double LAUNCHER_MIN_VELOCITY = 1000;     // minimum before launching
    private static final double FEED_TIME_SECONDS = 0.20;         // feeder run time per shot
    private static final double INTERVAL_BETWEEN_SHOTS = 1.0;     // seconds between shots
    private static final int TOTAL_BALLS = 3;                     // number of balls to launch

    // Hardware
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

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

        telemetry.addData("Status", "Ready to launch!");
        telemetry.addData("Strategy", "Launch %d balls with %.1fs intervals", TOTAL_BALLS, INTERVAL_BETWEEN_SHOTS);
        telemetry.update();

        // =====================================
        // WAIT FOR START
        // =====================================
        waitForStart();

        if (!opModeIsActive()) return;

        // =====================================
        // MAIN AUTONOMOUS SEQUENCE
        // =====================================
        telemetry.addData("Status", "Starting launch sequence");
        telemetry.update();

        // Start spinning up the launcher immediately
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Wait for launcher to reach minimum velocity
        telemetry.addData("Status", "Spinning up launcher...");
        telemetry.update();
        while (opModeIsActive() && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
            telemetry.addData("Launcher Speed", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_MIN_VELOCITY);
            telemetry.update();
            sleep(50);
        }

        // Launch each ball with interval
        for (int ball = 1; ball <= TOTAL_BALLS && opModeIsActive(); ball++) {
            telemetry.addData("Status", "Launching ball %d of %d", ball, TOTAL_BALLS);
            telemetry.addData("Launcher Speed", "%.0f", launcher.getVelocity());
            telemetry.update();

            // Ensure launcher is still at speed before feeding
            if (launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                // Wait for launcher to spin back up
                while (opModeIsActive() && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                    sleep(10);
                }
            }

            // Feed the ball
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(1.0);
            sleep((long) (FEED_TIME_SECONDS * 1000));

            // Stop feeders
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);

            // Wait interval before next shot (except after last ball)
            if (ball < TOTAL_BALLS) {
                telemetry.addData("Status", "Waiting for next shot...");
                telemetry.update();
                sleep((long) (INTERVAL_BETWEEN_SHOTS * 1000));
            }
        }

        // =====================================
        // CLEANUP
        // =====================================
        // Stop launcher
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Status", "Launch sequence complete!");
        telemetry.update();

        // Keep telemetry visible briefly before OpMode ends
        sleep(500);
    }
}
