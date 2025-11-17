/* Copyright (c) 2021 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an ENHANCED 2-motor tank drive OpMode with features from StarterBotTeleop.
 *
 * CURRENT CONFIGURATION: 2-MOTOR TANK DRIVE
 * This code uses 2 motors for differential/tank drive:
 * - Forward/Backward: Left stick Y-axis
 * - Rotation: Right stick X-axis
 * - Classic arcade-style control
 *
 * NOTE: This robot uses mecanum wheels with only 2 motors. With this setup, the mecanum
 * wheels function like regular wheels - you will NOT have omnidirectional strafing movement.
 * To get full mecanum capabilities (strafing), you would need to upgrade to 4 motors.
 *
 * ENHANCED FEATURES:
 * - BRAKE mode for precise stopping and control
 * - RUN_USING_ENCODER mode for consistent velocity control
 * - Speed control: Left/Right bumpers for slow/normal/turbo modes
 * - Enhanced telemetry showing power levels and motor velocities
 * - LinearOpMode structure for easier sequential programming
 *
 * MOTOR DIRECTIONS:
 * Test motor directions first! Push left stick forward - robot should move forward.
 * If not, flip motor directions in the code below.
 *
 * UPGRADE PATH:
 * To upgrade to full 4-motor mecanum drive, you'll need to add front_left_drive,
 * back_left_drive, front_right_drive, and back_right_drive motors and use the
 * full mecanum control algorithm.
 */

@TeleOp(name="Enhanced 2-Motor Drive", group="TeamCode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Speed control multipliers
    private static final double SLOW_MODE_MULTIPLIER = 0.3;   // Slow mode for precise movements
    private static final double NORMAL_SPEED = 0.8;           // Normal driving speed
    private static final double TURBO_MODE_MULTIPLIER = 1.0;  // Maximum speed

    // Declare OpMode members for the 2 motors
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    // Speed mode tracking
    private double speedMultiplier = NORMAL_SPEED;

    // Telemetry update rate (seconds between updates)
    private static final double TELEMETRY_UPDATE_RATE = 0.1; // 100ms = 10 updates/sec

    /**
     * Apply deadzone to joystick input to prevent drift from controller noise.
     * Values within the deadzone are treated as zero.
     * @param value The raw joystick input value (-1.0 to 1.0)
     * @param deadzone The deadzone threshold (typically 0.05)
     * @return The adjusted value, or 0.0 if within deadzone
     */
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        try {
            leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        } catch (IllegalArgumentException e) {
            telemetry.addLine("ERROR: Motor configuration missing!");
            telemetry.addLine("Please check that 'left_drive' and 'right_drive'");
            telemetry.addLine("are configured in the Robot Configuration.");
            telemetry.addData("Exception", e.getMessage());
            telemetry.update();
            sleep(5000); // Show error message for 5 seconds
            requestOpModeStop();
            return;
        }

        // Verify motors were initialized successfully
        if (leftDrive == null || rightDrive == null) {
            telemetry.addLine("ERROR: Motors not initialized!");
            telemetry.update();
            sleep(3000);
            requestOpModeStop();
            return;
        }

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motor on one side to be reversed to drive forward.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines
        // based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear
        // Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // ########################################################################################
        // ENHANCED FEATURES: Enable encoder-based velocity control and brake mode
        // ########################################################################################
        // Check if encoders are available and working
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100); // Allow time for encoder reset

        // Verify encoders are functioning
        boolean encodersAvailable = (leftDrive.getCurrentPosition() == 0 &&
                                     rightDrive.getCurrentPosition() == 0);

        if (encodersAvailable) {
            // Set motors to use encoders for consistent velocity control
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("✓ Encoders detected - Using velocity control");
        } else {
            // Fall back to direct power control if encoders not available
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("⚠ Encoders not detected - Using direct power control");
        }

        // Enable BRAKE mode for precise control and quick stopping
        // This causes the motor to slow down much faster when coasting, creating a much
        // more controllable drivetrain as the robot stops much quicker.
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized - 2 Motor Tank Drive");
        telemetry.addData("Controls", "Left Stick Y: Forward/Back | Right Stick X: Rotate");
        telemetry.addData("Speed Modes", "LB: Slow (30%) | Normal (80%) | RB: Turbo (100%)");
        telemetry.addData("Note", "Strafing NOT available (needs 4 motors)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ########################################################################################
            // SPEED CONTROL: Use bumpers to adjust driving speed
            // ########################################################################################
            if (gamepad1.left_bumper) {
                speedMultiplier = SLOW_MODE_MULTIPLIER;  // Slow mode for precision
            } else if (gamepad1.right_bumper) {
                speedMultiplier = TURBO_MODE_MULTIPLIER; // Turbo mode for speed
            } else {
                speedMultiplier = NORMAL_SPEED;          // Normal mode
            }

            // ########################################################################################
            // ARCADE DRIVE: Left stick for forward/backward, right stick for rotation
            // ########################################################################################
            // Note: Pushing stick forward gives negative value, so we negate it
            // Apply deadzone to prevent controller drift (5% threshold)
            double forward = applyDeadzone(-gamepad1.left_stick_y, 0.05);  // Forward/backward motion
            double rotate  = applyDeadzone(gamepad1.right_stick_x, 0.05);   // Rotation (turning)

            // Calculate power for each side using arcade drive formula
            // Left side: forward + rotate (turning right increases left power)
            // Right side: forward - rotate (turning right decreases right power)
            double leftPower  = forward + rotate;
            double rightPower = forward - rotate;

            // Normalize the values so neither side exceeds 100%
            // This ensures that the robot maintains the desired motion ratio
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower  /= max;
                rightPower /= max;
            }

            // Apply speed multiplier for slow/normal/turbo modes
            leftPower  *= speedMultiplier;
            rightPower *= speedMultiplier;

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First make sure the motors are in the correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the motors move in the correct direction, re-comment this code.

            /*
            leftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad - test left motor
            rightPower = gamepad1.b ? 1.0 : 0.0;  // B gamepad - test right motor
            */

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // ########################################################################################
            // ENHANCED TELEMETRY: Show detailed status information (rate-limited)
            // ########################################################################################
            // Update telemetry at controlled rate to prevent performance impact
            if (telemetryTimer.seconds() > TELEMETRY_UPDATE_RATE) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());

                // Speed mode indicator
                String speedMode = "Normal";
                if (speedMultiplier == SLOW_MODE_MULTIPLIER) speedMode = "SLOW";
                else if (speedMultiplier == TURBO_MODE_MULTIPLIER) speedMode = "TURBO";
                telemetry.addData("Speed Mode", speedMode + " (%.0f%%)", speedMultiplier * 100);

                // Motor power levels
                telemetry.addData("Motor Power", "L: %5.2f | R: %5.2f", leftPower, rightPower);

                // Motor velocities (in ticks per second)
                telemetry.addData("Left Velocity", "%.0f tps", leftDrive.getVelocity());
                telemetry.addData("Right Velocity", "%.0f tps", rightDrive.getVelocity());

                // Joystick inputs for debugging
                telemetry.addData("Input", "Forward: %.2f | Rotate: %.2f", forward, rotate);

                // Control hints
                telemetry.addData("", "------- Controls -------");
                telemetry.addData("Drive", "Left Stick Y: Fwd/Back");
                telemetry.addData("Turn", "Right Stick X: Rotate");
                telemetry.addData("Speed", "LB: Slow | RB: Turbo");
                telemetry.addData("", "");
                telemetry.addData("Note", "2-motor tank drive only");
                telemetry.addData("", "No strafing (needs 4 motors)");

                telemetry.update();
                telemetryTimer.reset();
            }
        }
    }
}
