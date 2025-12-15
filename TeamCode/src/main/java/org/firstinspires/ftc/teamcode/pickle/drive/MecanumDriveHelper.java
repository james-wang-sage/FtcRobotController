package org.firstinspires.ftc.teamcode.pickle.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Helper class for holonomic mecanum drive control.
 *
 * <h2>What is Holonomic Drive?</h2>
 * <p>Holonomic drive means the robot can move in any direction (X, Y) while simultaneously
 * rotating. This is the key advantage of mecanum wheels over tank drive.</p>
 *
 * <h2>Field-Centric vs Robot-Centric</h2>
 * <ul>
 *   <li><b>Robot-centric:</b> "Forward" means the direction the robot is facing</li>
 *   <li><b>Field-centric:</b> "Forward" always means +Y on the field, regardless of robot heading</li>
 * </ul>
 *
 * <p>Field-centric control requires an IMU to know the robot's current heading.</p>
 *
 * <h2>Usage Example:</h2>
 * <pre>
 * MecanumDriveHelper drive = new MecanumDriveHelper(fl, fr, bl, br, imu);
 *
 * // Drive toward a field position while rotating to face the goal
 * drive.driveFieldCentric(0.5, 0.3, 0.2); // 50% forward, 30% right, 20% rotation
 *
 * // Or drive directly to a pose
 * drive.driveToPose(targetPose, currentPose, maxSpeed);
 * </pre>
 */
public class MecanumDriveHelper {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;

    // Tuning constants
    private double strafeMultiplier = 1.1; // Compensate for strafe inefficiency
    private double headingKp = 0.02;       // Proportional gain for heading correction
    private double translationKp = 0.03;   // Proportional gain for position correction

    /**
     * Creates a new MecanumDriveHelper.
     *
     * @param frontLeft  Front left motor
     * @param frontRight Front right motor
     * @param backLeft   Back left motor
     * @param backRight  Back right motor
     * @param imu        IMU for heading reference (can be null for robot-centric only)
     */
    public MecanumDriveHelper(DcMotor frontLeft, DcMotor frontRight,
                               DcMotor backLeft, DcMotor backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
    }

    /**
     * Sets the strafe multiplier to compensate for lateral movement inefficiency.
     * Typical values are 1.1 to 1.4.
     *
     * @param multiplier The strafe correction factor
     */
    public void setStrafeMultiplier(double multiplier) {
        this.strafeMultiplier = multiplier;
    }

    /**
     * Robot-centric mecanum drive.
     *
     * <p>Control inputs are relative to the robot's current orientation:</p>
     * <ul>
     *   <li>+drive = forward (robot's front)</li>
     *   <li>+strafe = right (robot's right side)</li>
     *   <li>+rotate = clockwise rotation</li>
     * </ul>
     *
     * @param drive   Forward/backward power (-1 to 1)
     * @param strafe  Left/right power (-1 to 1, positive = right)
     * @param rotate  Rotation power (-1 to 1, positive = clockwise)
     */
    public void driveRobotCentric(double drive, double strafe, double rotate) {
        // Apply strafe correction for roller geometry
        strafe *= strafeMultiplier;

        // Mecanum kinematic equations
        double frontLeftPower  = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower   = drive - strafe + rotate;
        double backRightPower  = drive + strafe - rotate;

        // Normalize if any power exceeds 1.0
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));

        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }

    /**
     * Field-centric mecanum drive.
     *
     * <p>Control inputs are relative to the field coordinate system:</p>
     * <ul>
     *   <li>+fieldX = toward Red alliance (+X on field)</li>
     *   <li>+fieldY = toward far side (+Y on field)</li>
     *   <li>+rotate = clockwise rotation</li>
     * </ul>
     *
     * <p>The IMU heading is used to rotate the input vector so the robot moves
     * in the correct field direction regardless of its current heading.</p>
     *
     * @param fieldX  Field X power (-1 to 1)
     * @param fieldY  Field Y power (-1 to 1)
     * @param rotate  Rotation power (-1 to 1, positive = clockwise)
     */
    public void driveFieldCentric(double fieldX, double fieldY, double rotate) {
        if (imu == null) {
            // Fallback to robot-centric if no IMU
            driveRobotCentric(fieldY, fieldX, rotate);
            return;
        }

        // Get current heading from IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double headingRad = Math.toRadians(orientation.getYaw(AngleUnit.DEGREES));

        // Rotate field inputs by negative heading to get robot-relative inputs
        // This transforms field coordinates to robot coordinates
        double robotDrive  =  fieldY * Math.cos(headingRad) + fieldX * Math.sin(headingRad);
        double robotStrafe = -fieldY * Math.sin(headingRad) + fieldX * Math.cos(headingRad);

        driveRobotCentric(robotDrive, robotStrafe, rotate);
    }

    /**
     * Drives toward a target pose using proportional control.
     *
     * <p>This method calculates the error between current and target pose,
     * then applies proportional power to reduce the error. Call this method
     * repeatedly in a loop until {@link #isAtPose} returns true.</p>
     *
     * @param target    Target pose (position + heading)
     * @param current   Current estimated pose
     * @param maxSpeed  Maximum speed (0 to 1)
     */
    public void driveToPose(Pose2d target, Pose2d current, double maxSpeed) {
        // Calculate position error
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();

        // Calculate heading error (normalized to -PI to PI)
        double headingError = Pose2d.angleDifference(current.getHeading(), target.getHeading());

        // Apply proportional control
        double fieldX = clamp(errorX * translationKp, -maxSpeed, maxSpeed);
        double fieldY = clamp(errorY * translationKp, -maxSpeed, maxSpeed);
        double rotate = clamp(headingError * headingKp, -maxSpeed, maxSpeed);

        driveFieldCentric(fieldX, fieldY, rotate);
    }

    /**
     * Drives toward a target position while maintaining current heading.
     *
     * @param targetPosition Target position
     * @param currentPose    Current estimated pose
     * @param maxSpeed       Maximum speed (0 to 1)
     */
    public void driveToPosition(Translation2d targetPosition, Pose2d currentPose, double maxSpeed) {
        Pose2d targetPose = new Pose2d(targetPosition, currentPose.getHeading());
        driveToPose(targetPose, currentPose, maxSpeed);
    }

    /**
     * Drives toward a target position while rotating to face it.
     *
     * <p>This is useful for approaching the goal while orienting the launcher.</p>
     *
     * @param targetPosition Target position
     * @param currentPose    Current estimated pose
     * @param maxSpeed       Maximum speed (0 to 1)
     */
    public void driveToPositionFacingTarget(Translation2d targetPosition, Pose2d currentPose, double maxSpeed) {
        // Calculate heading that would face the target
        double dx = targetPosition.getX() - currentPose.getX();
        double dy = targetPosition.getY() - currentPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        Pose2d targetPose = new Pose2d(targetPosition, targetHeading);
        driveToPose(targetPose, currentPose, maxSpeed);
    }

    /**
     * Checks if the robot is at the target pose within tolerance.
     *
     * @param target            Target pose
     * @param current           Current pose
     * @param positionTolerance Position tolerance in mm
     * @param headingTolerance  Heading tolerance in radians
     * @return true if within tolerance
     */
    public boolean isAtPose(Pose2d target, Pose2d current,
                            double positionTolerance, double headingTolerance) {
        double positionError = current.distanceTo(target);
        double headingError = Math.abs(Pose2d.angleDifference(current.getHeading(), target.getHeading()));

        return positionError <= positionTolerance && headingError <= headingTolerance;
    }

    /**
     * Checks if the robot is at the target position within tolerance (ignoring heading).
     *
     * @param targetPosition    Target position
     * @param currentPose       Current pose
     * @param positionTolerance Position tolerance in mm
     * @return true if within tolerance
     */
    public boolean isAtPosition(Translation2d targetPosition, Pose2d currentPose, double positionTolerance) {
        double error = currentPose.getTranslation().distanceTo(targetPosition);
        return error <= positionTolerance;
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Sets the proportional gains for closed-loop control.
     *
     * @param translationKp Position proportional gain (power per mm error)
     * @param headingKp     Heading proportional gain (power per radian error)
     */
    public void setPIDGains(double translationKp, double headingKp) {
        this.translationKp = translationKp;
        this.headingKp = headingKp;
    }

    /**
     * Gets the current heading from the IMU.
     *
     * @return Heading in radians, or 0 if IMU is not available
     */
    public double getCurrentHeading() {
        if (imu == null) {
            return 0;
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return Math.toRadians(orientation.getYaw(AngleUnit.DEGREES));
    }

    /**
     * Creates a simple pose estimate using IMU heading and a base position.
     * For full odometry, you would also track X/Y position using encoders.
     *
     * @param basePosition Known starting position
     * @return Pose with current IMU heading
     */
    public Pose2d getSimplePose(Translation2d basePosition) {
        return new Pose2d(basePosition, getCurrentHeading());
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
