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
import org.firstinspires.ftc.teamcode.pickle.odometry.MecanumOdometry;
import org.firstinspires.ftc.teamcode.pickle.path.PathFollower;
import org.firstinspires.ftc.teamcode.pickle.path.Waypoint;
import org.firstinspires.ftc.teamcode.pickle.vision.AprilTagLocalizer;

/**
 * Advanced Holonomic Autonomous for DECODE with full sensor fusion.
 *
 * <h2>Improvements Over Basic PickleAutoOp</h2>
 * <ol>
 *   <li><b>Encoder-based Odometry:</b> Tracks robot position using wheel encoders,
 *       providing continuous position updates between AprilTag sightings.</li>
 *   <li><b>AprilTag Position Correction:</b> Uses goal AprilTags to correct accumulated
 *       odometry drift, providing accurate absolute positioning.</li>
 *   <li><b>Road Runner-style Path Following:</b> Smooth waypoint-based navigation with
 *       configurable heading modes and action callbacks.</li>
 * </ol>
 *
 * <h2>Sensor Fusion Architecture</h2>
 * <pre>
 * ┌─────────────┐     ┌─────────────────┐     ┌──────────────┐
 * │   Wheel     │────▶│   Odometry      │────▶│   Pose       │
 * │  Encoders   │     │  Integration    │     │  Estimate    │
 * └─────────────┘     └─────────────────┘     └──────┬───────┘
 *                                                     │
 * ┌─────────────┐     ┌─────────────────┐            │
 * │  AprilTag   │────▶│   Position      │────────────┤ Correction
 * │   Camera    │     │   Measurement   │            │
 * └─────────────┘     └─────────────────┘            ▼
 *                                              ┌──────────────┐
 *                                              │   Fused      │
 *                                              │   Position   │
 *                                              └──────────────┘
 * </pre>
 *
 * <h2>Path Following Example</h2>
 * <pre>
 * The robot follows a smooth path:
 *
 *   START ──▶ Waypoint 1 ──▶ Waypoint 2 ──▶ SHOOTING ──▶ PARK
 *     ↓           ↓              ↓              ↓           ↓
 *   (init)   (approach)    (align to      (fire 3      (backup)
 *                           goal)          balls)
 * </pre>
 *
 * <h2>Tuning Parameters</h2>
 * <p>This OpMode exposes several tunable parameters for optimal performance.
 * Start with the defaults and adjust based on robot testing.</p>
 */
@Autonomous(name = "PickleAutoHolonomic", group = "StarterBot")
public class PickleAutoHolonomic extends OpMode {

    // ========== TUNING CONSTANTS ==========

    // Drive speeds
    /** Maximum drive speed during path following */
    final double MAX_DRIVE_SPEED = 0.6;

    /** Slower speed when approaching goal for precision */
    final double APPROACH_SPEED = 0.35;

    /** Speed for final alignment before shooting */
    final double ALIGN_SPEED = 0.25;

    // Tolerances
    /** Position tolerance in mm (how close is "close enough") */
    final double POSITION_TOLERANCE_MM = 40.0;

    /** Heading tolerance in degrees */
    final double HEADING_TOLERANCE_DEG = 3.0;

    /** Maximum time to reach each waypoint before giving up */
    final double WAYPOINT_TIMEOUT_SEC = 4.0;

    /** Time to hold at shooting position before firing */
    final double SETTLE_TIME_SEC = 0.3;

    // AprilTag fusion settings
    /** How much to trust AprilTag vs odometry (0=ignore, 1=full trust) */
    final double APRILTAG_BLEND_FACTOR = 0.4;

    /** Maximum age of AprilTag reading to use (ms) */
    final long APRILTAG_MAX_AGE_MS = 200;

    /** Distance at which we start trusting AprilTags more */
    final double APRILTAG_CLOSE_RANGE_MM = 1500;

    // Odometry tuning (for GoBILDA 312 RPM motors with mecanum wheels)
    final double WHEEL_RADIUS_MM = 48.0;         // GoBILDA mecanum wheel
    final double TICKS_PER_REV = 537.7;          // GoBILDA 312 RPM motor encoder
    final double TRACK_WIDTH_MM = 350.0;         // Left-right wheel distance
    final double WHEEL_BASE_MM = 300.0;          // Front-back wheel distance

    // Launcher constants
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double FEED_TIME = 0.20;
    final double TIME_BETWEEN_SHOTS = 2;

    // ========== HARDWARE ==========

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private IMU imu;

    // ========== SUBSYSTEMS ==========

    private MecanumDriveHelper driveHelper;
    private MecanumOdometry odometry;
    private AprilTagLocalizer aprilTagLocalizer;
    private PathFollower pathFollower;

    // ========== STATE ==========

    private enum AutoState {
        FOLLOW_PATH_TO_SHOOTING,
        SETTLE_AT_POSITION,
        REQUEST_SHOT,
        WAIT_FOR_SHOT,
        FOLLOW_PATH_TO_PARK,
        COMPLETE
    }

    private enum LaunchState { IDLE, PREPARE, LAUNCH }

    private AutoState autoState = AutoState.FOLLOW_PATH_TO_SHOOTING;
    private LaunchState launchState = LaunchState.IDLE;
    private Alliance alliance = Alliance.RED;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();

    private int shotsToFire = 3;

    // Target positions
    private Pose2d startPose;
    private Translation2d shootingPosition;
    private Translation2d parkPosition;
    private Translation2d goalCenter;

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

        // Motor directions
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

        // Use encoders for odometry - IMPORTANT: Must be RUN_WITHOUT_ENCODER for
        // direct power control, but we still read encoder values for odometry
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // Initialize subsystems
        initSubsystems();

        telemetry.addData("Status", "Initialized - Advanced Holonomic Mode");
        telemetry.addData("IMU", imu != null ? "Ready" : "FAILED!");
        telemetry.addData("AprilTag", aprilTagLocalizer != null ? "Ready" : "Not available");
    }

    private void initSubsystems() {
        // Create drive helper
        driveHelper = new MecanumDriveHelper(frontLeft, frontRight, backLeft, backRight, imu);
        driveHelper.setStrafeMultiplier(1.2);
        driveHelper.setPIDGains(0.003, 2.0); // Tuned for faster response

        // Create odometry
        odometry = new MecanumOdometry(frontLeft, frontRight, backLeft, backRight, imu);
        odometry.setWheelParameters(WHEEL_RADIUS_MM, TICKS_PER_REV, TRACK_WIDTH_MM, WHEEL_BASE_MM);
        odometry.setUseIMUHeading(true);

        // Create AprilTag localizer
        try {
            aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, PickleHardwareNames.WEBCAM_NAME);
            // Configure camera position on robot (adjust these for your robot!)
            aprilTagLocalizer.setCameraPosition(
                    FieldConstants.inchesToMm(6),  // 6 inches forward of center
                    0,                              // Centered left-right
                    FieldConstants.inchesToMm(8)   // 8 inches up
            );
            aprilTagLocalizer.setMaxDetectionRange(FieldConstants.inchesToMm(72)); // 6 feet max
        } catch (Exception e) {
            aprilTagLocalizer = null;
            telemetry.addData("AprilTag Error", e.getMessage());
        }

        // Create path follower
        pathFollower = new PathFollower(driveHelper);
        pathFollower.setDefaultSpeed(MAX_DRIVE_SPEED);
        pathFollower.setWaypointTimeout(WAYPOINT_TIMEOUT_SEC);
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

        // Test AprilTag detection
        int tagCount = 0;
        if (aprilTagLocalizer != null) {
            tagCount = aprilTagLocalizer.getDetections().size();
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal Tag ID", DecodeField.getGoalAprilTagId(alliance));
        telemetry.addLine();
        telemetry.addData("Start Pose", startPose);
        telemetry.addData("Shooting Position", shootingPosition);
        telemetry.addData("AprilTags Visible", tagCount);
    }

    @Override
    public void start() {
        // Set initial pose
        calculateTargetPositions();
        odometry.resetPose(startPose);
        imu.resetYaw();

        // Build path to shooting position
        buildShootingPath();

        autoState = AutoState.FOLLOW_PATH_TO_SHOOTING;
        launchState = LaunchState.IDLE;
        shotsToFire = 3;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        // Update odometry
        odometry.update();
        Pose2d currentPose = odometry.getPose();

        // Apply AprilTag corrections
        applyAprilTagCorrection();

        // State machine
        switch (autoState) {
            case FOLLOW_PATH_TO_SHOOTING:
                handleFollowPathToShooting();
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
                        buildParkPath();
                        stateTimer.reset();
                        autoState = AutoState.FOLLOW_PATH_TO_PARK;
                    }
                }
                break;

            case FOLLOW_PATH_TO_PARK:
                handleFollowPathToPark();
                break;

            case COMPLETE:
                driveHelper.stop();
                break;
        }

        // Telemetry
        updateTelemetry(currentPose);
    }

    @Override
    public void stop() {
        // Clean up resources
        if (aprilTagLocalizer != null) {
            aprilTagLocalizer.close();
        }
    }

    // ========== STATE HANDLERS ==========

    private void handleFollowPathToShooting() {
        Pose2d currentPose = odometry.getPose();

        // Update path follower
        pathFollower.update(currentPose);

        // Check if path is complete
        if (pathFollower.isComplete()) {
            driveHelper.stop();
            stateTimer.reset();
            autoState = AutoState.SETTLE_AT_POSITION;
        }
    }

    private void handleSettle() {
        // Hold position while settling
        Pose2d currentPose = odometry.getPose();
        double headingToGoal = DecodeField.headingToGoal(currentPose.getTranslation(), alliance);
        Pose2d targetPose = new Pose2d(shootingPosition, headingToGoal);

        // Fine-tune alignment
        driveHelper.driveToPose(targetPose, currentPose, ALIGN_SPEED);

        if (stateTimer.seconds() > SETTLE_TIME_SEC) {
            driveHelper.stop();
            autoState = AutoState.REQUEST_SHOT;
        }
    }

    private void handleFollowPathToPark() {
        Pose2d currentPose = odometry.getPose();

        // Update path follower
        pathFollower.update(currentPose);

        // Check if path is complete or timeout
        if (pathFollower.isComplete() || stateTimer.seconds() > 5.0) {
            driveHelper.stop();
            autoState = AutoState.COMPLETE;
        }
    }

    // ========== PATH BUILDING ==========

    private void buildShootingPath() {
        Pose2d currentPose = odometry.getPose();

        // Build path using fluent API
        pathFollower.buildPath()
                .from(currentPose)
                .setSpeed(MAX_DRIVE_SPEED)
                .setTolerance(POSITION_TOLERANCE_MM, Math.toRadians(HEADING_TOLERANCE_DEG))
                // Move toward shooting position while facing the goal
                .lineToFacing(
                        shootingPosition.getX(),
                        shootingPosition.getY(),
                        goalCenter.getX(),
                        goalCenter.getY()
                )
                .build();
    }

    private void buildParkPath() {
        Pose2d currentPose = odometry.getPose();

        // Simple backup path to park
        pathFollower.buildPath()
                .from(currentPose)
                .setSpeed(APPROACH_SPEED)
                .lineTo(parkPosition.getX(), parkPosition.getY())
                .build();
    }

    // ========== SENSOR FUSION ==========

    /**
     * Applies AprilTag corrections to odometry.
     *
     * <p>This implements a simple sensor fusion approach:</p>
     * <ul>
     *   <li>Odometry provides continuous high-frequency updates</li>
     *   <li>AprilTag provides periodic absolute corrections</li>
     *   <li>Blend factor determines trust level (higher when closer to tag)</li>
     * </ul>
     */
    private void applyAprilTagCorrection() {
        if (aprilTagLocalizer == null) return;

        Pose2d visionPose = aprilTagLocalizer.getEstimatedPose(alliance);

        if (visionPose != null && aprilTagLocalizer.hasRecentDetection(APRILTAG_MAX_AGE_MS)) {
            // Calculate adaptive blend factor based on distance to goal
            Pose2d currentPose = odometry.getPose();
            double distanceToGoal = currentPose.getTranslation().distanceTo(goalCenter);

            double blendFactor = APRILTAG_BLEND_FACTOR;
            if (distanceToGoal < APRILTAG_CLOSE_RANGE_MM) {
                // Trust AprilTag more when close to goal
                blendFactor = APRILTAG_BLEND_FACTOR * 1.5;
            }

            // Apply correction
            odometry.correctPose(visionPose, blendFactor);
        }
    }

    // ========== HELPER METHODS ==========

    private void calculateTargetPositions() {
        // Get starting position from DecodeField
        startPose = DecodeField.getStartingPose(alliance, 1);

        // Get goal center for heading calculations
        goalCenter = DecodeField.getGoalCenter(alliance);

        // Calculate shooting position (offset from goal for launcher distance)
        double shootingDistanceMM = FieldConstants.inchesToMm(24.0); // 24 inches from goal

        // Offset toward field center from goal
        double angle = Math.atan2(-goalCenter.getY(), -goalCenter.getX()); // Toward center
        shootingPosition = new Translation2d(
                goalCenter.getX() + shootingDistanceMM * Math.cos(angle),
                goalCenter.getY() + shootingDistanceMM * Math.sin(angle)
        );

        // Park position (back toward starting area)
        parkPosition = new Translation2d(
                shootingPosition.getX() + FieldConstants.inchesToMm(-18) * Math.cos(angle),
                shootingPosition.getY() + FieldConstants.inchesToMm(-18) * Math.sin(angle)
        );
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

    private void updateTelemetry(Pose2d currentPose) {
        telemetry.addData("State", autoState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shots Remaining", shotsToFire);
        telemetry.addLine();

        // Pose information
        telemetry.addData("Pose", String.format("(%.0f, %.0f) @ %.1f°",
                currentPose.getX(), currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())));

        // Path progress
        if (pathFollower != null && !pathFollower.isComplete()) {
            telemetry.addData("Path Progress", String.format("%d/%d (%.0f%%)",
                    pathFollower.getCurrentWaypointIndex() + 1,
                    pathFollower.getWaypointCount(),
                    pathFollower.getProgress() * 100));
        }

        // Distance to target
        if (shootingPosition != null) {
            double distance = currentPose.getTranslation().distanceTo(shootingPosition);
            telemetry.addData("Distance to Target", String.format("%.1f mm", distance));
        }

        // AprilTag status
        if (aprilTagLocalizer != null) {
            telemetry.addData("AprilTag", aprilTagLocalizer.getDiagnostics());
        }

        // Odometry diagnostics
        telemetry.addData("Encoders", odometry.getDiagnostics());

        telemetry.update();
    }
}
