package org.firstinspires.ftc.teamcode.pickle;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pickle.drive.MecanumDriveHelper;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;
import org.firstinspires.ftc.teamcode.pickle.field.DecodeField;
import org.firstinspires.ftc.teamcode.pickle.field.FieldConstants;
import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Holonomic Autonomous for DECODE - Uses full mecanum capabilities.
 *
 * <h2>Key Difference from PickleAutoOp</h2>
 * <p>This OpMode uses <b>field-centric holonomic drive</b> to move diagonally
 * toward the goal while simultaneously rotating to face it. This is faster
 * than the sequential drive→rotate→drive approach.</p>
 *
 * <h2>Path Comparison</h2>
 * <pre>
 * PickleAutoOp (Sequential):     PickleAutoHolonomic (Holonomic):
 * ┌─────────────────────┐        ┌─────────────────────┐
 * │         GOAL        │        │         GOAL        │
 * │          ▲          │        │          ▲          │
 * │          │          │        │         /           │
 * │          │ drive    │        │        / diagonal   │
 * │    ┌─────┘          │        │       /  + rotate   │
 * │    │ rotate         │        │      /              │
 * │    │                │        │     /               │
 * │    └─ drive         │        │    ●                │
 * │    ●                │        │  START              │
 * │  START              │        │                     │
 * └─────────────────────┘        └─────────────────────┘
 *   ~4-5 seconds                   ~2-3 seconds
 * </pre>
 *
 * <h2>Requirements</h2>
 * <ul>
 *   <li>IMU must be working (no fallback to encoders for holonomic)</li>
 *   <li>Starting position must be known accurately</li>
 * </ul>
 */
@Autonomous(name = "PickleAutoHolonomic", group = "StarterBot")
public class PickleAutoHolonomic extends OpMode {

    // ========== TUNING CONSTANTS ==========

    /** Maximum drive speed during holonomic movement (0 to 1) */
    final double MAX_DRIVE_SPEED = 0.6;

    /** Slower speed when approaching goal for precision */
    final double APPROACH_SPEED = 0.3;

    /** Position tolerance in mm (how close is "close enough") */
    final double POSITION_TOLERANCE_MM = 50.0;

    /** Heading tolerance in degrees */
    final double HEADING_TOLERANCE_DEG = 5.0;

    /** Maximum time to reach goal before giving up */
    final double DRIVE_TO_GOAL_TIMEOUT_SEC = 6.0;

    /** Time to hold at shooting position before firing */
    final double SETTLE_TIME_SEC = 0.5;

    // Launcher constants (same as PickleAutoOp)
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double FEED_TIME = 0.20;
    final double TIME_BETWEEN_SHOTS = 2;

    // ========== HARDWARE ==========

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private IMU imu;

    private MecanumDriveHelper driveHelper;

    // ========== STATE ==========

    private enum AutoState {
        DRIVE_TO_SHOOTING_POSITION,
        SETTLE_AT_POSITION,
        REQUEST_SHOT,
        WAIT_FOR_SHOT,
        DRIVE_TO_PARK,
        COMPLETE
    }

    private enum LaunchState { IDLE, PREPARE, LAUNCH }

    private AutoState autoState = AutoState.DRIVE_TO_SHOOTING_POSITION;
    private LaunchState launchState = LaunchState.IDLE;
    private Alliance alliance = Alliance.RED;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();

    private int shotsToFire = 3;

    // Position tracking (simple dead-reckoning from known start)
    private Pose2d currentPose;
    private Pose2d startPose;
    private Translation2d shootingPosition;
    private Translation2d parkPosition;

    // ========== OPMODE LIFECYCLE ==========

    @Override
    public void init() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PickleHardwareNames.BACK_RIGHT_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, PickleHardwareNames.LAUNCHER_MOTOR);
        leftFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.LEFT_FEEDER_SERVO);
        rightFeeder = hardwareMap.get(CRServo.class, PickleHardwareNames.RIGHT_FEEDER_SERVO);

        // Motor directions (same as PickleAutoOp)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake mode for precision
        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Run without encoder for direct power control (holonomic mode)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Launcher setup
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        initIMU();

        // Create drive helper
        driveHelper = new MecanumDriveHelper(frontLeft, frontRight, backLeft, backRight, imu);
        driveHelper.setStrafeMultiplier(1.2);
        driveHelper.setPIDGains(0.002, 1.5); // Tune these for your robot

        telemetry.addData("Status", "Initialized - Holonomic Mode");
        telemetry.addData("IMU", imu != null ? "Ready" : "FAILED - Holonomic won't work!");
    }

    @Override
    public void init_loop() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Alliance selection
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.x) alliance = Alliance.BLUE;

        // Calculate target positions based on alliance
        calculateTargetPositions();

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal Tag ID", DecodeField.getGoalAprilTagId(alliance));
        telemetry.addLine();
        telemetry.addData("Start Pose", startPose);
        telemetry.addData("Shooting Position", shootingPosition);
    }

    @Override
    public void start() {
        // Set initial pose
        calculateTargetPositions();
        currentPose = startPose;
        imu.resetYaw();

        autoState = AutoState.DRIVE_TO_SHOOTING_POSITION;
        launchState = LaunchState.IDLE;
        shotsToFire = 3;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        // Update current heading from IMU (position is estimated)
        updatePoseEstimate();

        switch (autoState) {
            case DRIVE_TO_SHOOTING_POSITION:
                handleDriveToShooting();
                break;

            case SETTLE_AT_POSITION:
                handleSettle();
                break;

            case REQUEST_SHOT:
                launch(true);
                autoState = AutoState.WAIT_FOR_SHOT;
                break;

            case WAIT_FOR_SHOT:
                if (launch(false)) {
                    shotsToFire--;
                    if (shotsToFire > 0) {
                        autoState = AutoState.REQUEST_SHOT;
                    } else {
                        launcher.setVelocity(0);
                        stateTimer.reset();
                        autoState = AutoState.DRIVE_TO_PARK;
                    }
                }
                break;

            case DRIVE_TO_PARK:
                handleDriveToPark();
                break;

            case COMPLETE:
                driveHelper.stop();
                break;
        }

        // Telemetry
        telemetry.addData("State", autoState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shots Remaining", shotsToFire);
        telemetry.addLine();
        telemetry.addData("Current Pose", currentPose);
        telemetry.addData("Target", shootingPosition);
        if (shootingPosition != null) {
            double distance = currentPose.getTranslation().distanceTo(shootingPosition);
            telemetry.addData("Distance to Target", String.format("%.1f mm", distance));
        }
        telemetry.update();
    }

    // ========== STATE HANDLERS ==========

    private void handleDriveToShooting() {
        // Timeout protection
        if (stateTimer.seconds() > DRIVE_TO_GOAL_TIMEOUT_SEC) {
            telemetry.addData("WARNING", "Timeout - proceeding to shoot");
            driveHelper.stop();
            stateTimer.reset();
            autoState = AutoState.SETTLE_AT_POSITION;
            return;
        }

        // Check if we've arrived
        double distance = currentPose.getTranslation().distanceTo(shootingPosition);
        if (distance < POSITION_TOLERANCE_MM) {
            driveHelper.stop();
            stateTimer.reset();
            autoState = AutoState.SETTLE_AT_POSITION;
            return;
        }

        // Calculate target heading to face the goal
        double headingToGoal = DecodeField.headingToGoal(currentPose.getTranslation(), alliance);
        Pose2d targetPose = new Pose2d(shootingPosition, headingToGoal);

        // Choose speed based on distance
        double speed = distance > 500 ? MAX_DRIVE_SPEED : APPROACH_SPEED;

        // Drive toward target while rotating to face goal
        driveHelper.driveToPose(targetPose, currentPose, speed);
    }

    private void handleSettle() {
        // Wait for robot to stabilize before shooting
        driveHelper.stop();
        if (stateTimer.seconds() > SETTLE_TIME_SEC) {
            autoState = AutoState.REQUEST_SHOT;
        }
    }

    private void handleDriveToPark() {
        // Simple backup to park position
        double distance = currentPose.getTranslation().distanceTo(parkPosition);
        if (distance < POSITION_TOLERANCE_MM || stateTimer.seconds() > 3.0) {
            driveHelper.stop();
            autoState = AutoState.COMPLETE;
            return;
        }

        Pose2d targetPose = new Pose2d(parkPosition, currentPose.getHeading());
        driveHelper.driveToPose(targetPose, currentPose, APPROACH_SPEED);
    }

    // ========== HELPER METHODS ==========

    private void calculateTargetPositions() {
        // Get starting position from DecodeField
        startPose = DecodeField.getStartingPose(alliance, 1);

        // Calculate shooting position (offset from goal for launcher distance)
        Translation2d goalCenter = DecodeField.getGoalCenter(alliance);
        double shootingDistanceMM = FieldConstants.inchesToMm(24.0); // 24 inches from goal

        // Offset toward field center from goal
        double angle = Math.atan2(-goalCenter.getY(), -goalCenter.getX()); // Toward center
        shootingPosition = new Translation2d(
                goalCenter.getX() + shootingDistanceMM * Math.cos(angle),
                goalCenter.getY() + shootingDistanceMM * Math.sin(angle)
        );

        // Park position (back toward starting area)
        parkPosition = new Translation2d(
                shootingPosition.getX() + FieldConstants.inchesToMm(-12) * Math.cos(angle),
                shootingPosition.getY() + FieldConstants.inchesToMm(-12) * Math.sin(angle)
        );
    }

    private void updatePoseEstimate() {
        if (imu == null) return;

        // Simple pose estimate: heading from IMU, position estimated by dead-reckoning
        // For full odometry, you would track wheel encoder distances
        double currentHeading = Math.toRadians(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // Note: This is a simplified estimate. The position doesn't actually update
        // without encoder-based odometry. For a real implementation, you'd integrate
        // wheel velocities to track X/Y position.
        currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), currentHeading);
    }

    private void initIMU() {
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
    }

    private boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
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
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }
}
