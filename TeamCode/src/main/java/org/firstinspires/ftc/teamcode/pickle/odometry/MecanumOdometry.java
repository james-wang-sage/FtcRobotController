package org.firstinspires.ftc.teamcode.pickle.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Encoder-based odometry for mecanum drivetrains.
 *
 * <h2>What is Odometry?</h2>
 * <p>Odometry is the process of estimating the robot's position by integrating
 * wheel movements over time. For mecanum wheels, we use all four wheel encoders
 * to calculate X, Y, and rotation changes.</p>
 *
 * <h2>How Mecanum Odometry Works</h2>
 * <p>Each mecanum wheel contributes to motion in a specific way:</p>
 * <pre>
 *   Front Left (FL)  →  +X, +Y, +θ
 *   Front Right (FR) →  +X, -Y, -θ
 *   Back Left (BL)   →  +X, -Y, +θ
 *   Back Right (BR)  →  +X, +Y, -θ
 *
 * Forward kinematics:
 *   ΔX = (FL + FR + BL + BR) / 4
 *   ΔY = (-FL + FR + BL - BR) / 4
 *   Δθ = (-FL + FR - BL + BR) / (4 * trackWidth)
 * </pre>
 *
 * <h2>Coordinate System</h2>
 * <ul>
 *   <li>+X = Forward (robot's front direction)</li>
 *   <li>+Y = Left (robot's left side)</li>
 *   <li>+θ = Counter-clockwise rotation</li>
 * </ul>
 *
 * <h2>Units</h2>
 * <p>All distances are in millimeters, angles in radians.</p>
 *
 * <h2>Usage</h2>
 * <pre>
 * // In init()
 * odometry = new MecanumOdometry(fl, fr, bl, br, imu);
 * odometry.setWheelParameters(WHEEL_RADIUS_MM, TICKS_PER_REV, TRACK_WIDTH_MM, WHEEL_BASE_MM);
 * odometry.resetPose(startingPose);
 *
 * // In loop()
 * odometry.update();
 * Pose2d currentPose = odometry.getPose();
 * </pre>
 */
public class MecanumOdometry {

    // Hardware
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;

    // Wheel geometry parameters
    private double wheelRadiusMM = 48.0;        // GoBILDA mecanum wheel radius
    private double ticksPerRevolution = 537.7;  // GoBILDA 312 RPM motor
    private double trackWidthMM = 350.0;        // Distance between left and right wheels
    private double wheelBaseMM = 300.0;         // Distance between front and back wheels

    // Derived values
    private double mmPerTick;

    // Current pose estimate
    private Pose2d pose = new Pose2d(0, 0, 0);

    // Previous encoder values for delta calculation
    private int prevFL = 0;
    private int prevFR = 0;
    private int prevBL = 0;
    private int prevBR = 0;

    // For IMU-fused heading (more accurate than encoder-only)
    private boolean useIMUHeading = true;

    /**
     * Creates a new MecanumOdometry instance.
     *
     * @param frontLeft  Front left motor with encoder
     * @param frontRight Front right motor with encoder
     * @param backLeft   Back left motor with encoder
     * @param backRight  Back right motor with encoder
     * @param imu        IMU for heading fusion (can be null)
     */
    public MecanumOdometry(DcMotor frontLeft, DcMotor frontRight,
                           DcMotor backLeft, DcMotor backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;

        calculateDerivedValues();
    }

    /**
     * Sets the wheel parameters for accurate distance calculation.
     *
     * @param wheelRadiusMM     Wheel radius in mm
     * @param ticksPerRevolution Encoder ticks per wheel revolution
     * @param trackWidthMM      Distance between left and right wheels in mm
     * @param wheelBaseMM       Distance between front and back wheels in mm
     */
    public void setWheelParameters(double wheelRadiusMM, double ticksPerRevolution,
                                    double trackWidthMM, double wheelBaseMM) {
        this.wheelRadiusMM = wheelRadiusMM;
        this.ticksPerRevolution = ticksPerRevolution;
        this.trackWidthMM = trackWidthMM;
        this.wheelBaseMM = wheelBaseMM;
        calculateDerivedValues();
    }

    private void calculateDerivedValues() {
        // Distance traveled per encoder tick
        double wheelCircumference = 2 * Math.PI * wheelRadiusMM;
        mmPerTick = wheelCircumference / ticksPerRevolution;
    }

    /**
     * Sets whether to use IMU heading instead of encoder-derived heading.
     * IMU heading is generally more accurate but may drift over long periods.
     *
     * @param useIMU true to use IMU heading, false for encoder-only
     */
    public void setUseIMUHeading(boolean useIMU) {
        this.useIMUHeading = useIMU;
    }

    /**
     * Resets the pose to a known position.
     * Call this at the start of autonomous with the known starting position.
     *
     * @param newPose The known pose to reset to
     */
    public void resetPose(Pose2d newPose) {
        this.pose = newPose;
        resetEncoders();
    }

    /**
     * Resets encoder baseline without changing pose.
     */
    public void resetEncoders() {
        prevFL = frontLeft.getCurrentPosition();
        prevFR = frontRight.getCurrentPosition();
        prevBL = backLeft.getCurrentPosition();
        prevBR = backRight.getCurrentPosition();
    }

    /**
     * Updates the pose estimate based on encoder changes.
     * Call this every loop iteration for best accuracy.
     */
    public void update() {
        // Get current encoder positions
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        // Calculate deltas
        int deltaFL = currFL - prevFL;
        int deltaFR = currFR - prevFR;
        int deltaBL = currBL - prevBL;
        int deltaBR = currBR - prevBR;

        // Convert to mm
        double dFL = deltaFL * mmPerTick;
        double dFR = deltaFR * mmPerTick;
        double dBL = deltaBL * mmPerTick;
        double dBR = deltaBR * mmPerTick;

        // Mecanum forward kinematics (robot-relative deltas)
        // Note: Signs depend on motor direction configuration
        double robotDeltaX = (dFL + dFR + dBL + dBR) / 4.0;  // Forward
        double robotDeltaY = (-dFL + dFR + dBL - dBR) / 4.0; // Left strafe

        // Calculate heading change from encoders
        double encoderDeltaHeading = (-dFL + dFR - dBL + dBR) / (4.0 * trackWidthMM);

        // Get current heading
        double currentHeading;
        if (useIMUHeading && imu != null) {
            currentHeading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            currentHeading = pose.getHeading() + encoderDeltaHeading;
        }

        // Transform robot-relative deltas to field-relative
        double cos = Math.cos(pose.getHeading());
        double sin = Math.sin(pose.getHeading());

        double fieldDeltaX = robotDeltaX * cos - robotDeltaY * sin;
        double fieldDeltaY = robotDeltaX * sin + robotDeltaY * cos;

        // Update pose
        pose = new Pose2d(
                pose.getX() + fieldDeltaX,
                pose.getY() + fieldDeltaY,
                currentHeading
        );

        // Store current positions for next iteration
        prevFL = currFL;
        prevFR = currFR;
        prevBL = currBL;
        prevBR = currBR;
    }

    /**
     * Gets the current estimated pose.
     *
     * @return The current pose (x, y in mm, heading in radians)
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Gets the current position.
     *
     * @return The current position (x, y in mm)
     */
    public Translation2d getPosition() {
        return pose.getTranslation();
    }

    /**
     * Gets the current heading.
     *
     * @return The current heading in radians
     */
    public double getHeading() {
        return pose.getHeading();
    }

    /**
     * Corrects the pose with an external measurement (e.g., from AprilTag).
     * This is typically called when a vision system provides a position update.
     *
     * @param measuredPose The measured pose from an external source
     * @param blendFactor  How much to trust the measurement (0 = ignore, 1 = fully replace)
     */
    public void correctPose(Pose2d measuredPose, double blendFactor) {
        blendFactor = Math.max(0, Math.min(1, blendFactor));

        double newX = pose.getX() + (measuredPose.getX() - pose.getX()) * blendFactor;
        double newY = pose.getY() + (measuredPose.getY() - pose.getY()) * blendFactor;

        // Blend heading carefully to handle wrap-around
        double headingDiff = Pose2d.angleDifference(pose.getHeading(), measuredPose.getHeading());
        double newHeading = Pose2d.normalizeAngle(pose.getHeading() + headingDiff * blendFactor);

        pose = new Pose2d(newX, newY, newHeading);
    }

    /**
     * Directly sets the pose (use with caution, typically for AprilTag corrections).
     *
     * @param newPose The new pose to set
     */
    public void setPose(Pose2d newPose) {
        this.pose = newPose;
    }

    /**
     * Gets diagnostic information about the odometry state.
     *
     * @return Formatted string with encoder positions and pose
     */
    public String getDiagnostics() {
        return String.format(
                "FL:%d FR:%d BL:%d BR:%d | Pose:(%.1f, %.1f, %.1f°)",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())
        );
    }
}
