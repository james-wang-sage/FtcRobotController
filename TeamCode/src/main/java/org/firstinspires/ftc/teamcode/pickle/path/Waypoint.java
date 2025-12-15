package org.firstinspires.ftc.teamcode.pickle.path;

import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Represents a waypoint in a path.
 *
 * <p>A waypoint consists of:</p>
 * <ul>
 *   <li>Position (x, y) - where the robot should be</li>
 *   <li>Heading - which direction the robot should face (optional)</li>
 *   <li>Speed - how fast to travel to this waypoint</li>
 *   <li>Tolerance - how close is "close enough"</li>
 * </ul>
 *
 * <h2>Heading Modes</h2>
 * <ul>
 *   <li><b>FIXED:</b> Robot maintains a specific heading</li>
 *   <li><b>PATH:</b> Robot faces the direction of travel</li>
 *   <li><b>TARGET:</b> Robot faces a specific target point</li>
 * </ul>
 */
public class Waypoint {

    public enum HeadingMode {
        /** Robot maintains the specified heading */
        FIXED,
        /** Robot faces the direction it's traveling */
        PATH,
        /** Robot faces toward a target point (set with setHeadingTarget) */
        TARGET,
        /** Robot keeps its current heading unchanged */
        MAINTAIN
    }

    private final Translation2d position;
    private double heading = 0;
    private HeadingMode headingMode = HeadingMode.FIXED;
    private Translation2d headingTarget = null;

    private double speed = 0.5;
    private double positionTolerance = 50; // mm
    private double headingTolerance = Math.toRadians(5); // radians

    // Action to perform when waypoint is reached
    private Runnable action = null;
    private double actionDelay = 0; // seconds to wait after reaching before action

    /**
     * Creates a waypoint at the specified position.
     *
     * @param x X position in mm
     * @param y Y position in mm
     */
    public Waypoint(double x, double y) {
        this.position = new Translation2d(x, y);
    }

    /**
     * Creates a waypoint at the specified position with heading.
     *
     * @param x       X position in mm
     * @param y       Y position in mm
     * @param heading Heading in radians
     */
    public Waypoint(double x, double y, double heading) {
        this.position = new Translation2d(x, y);
        this.heading = heading;
        this.headingMode = HeadingMode.FIXED;
    }

    /**
     * Creates a waypoint from a Pose2d.
     *
     * @param pose The pose defining position and heading
     */
    public Waypoint(Pose2d pose) {
        this.position = pose.getTranslation();
        this.heading = pose.getHeading();
        this.headingMode = HeadingMode.FIXED;
    }

    /**
     * Creates a waypoint from a Translation2d.
     *
     * @param position The position
     */
    public Waypoint(Translation2d position) {
        this.position = position;
    }

    // ===== Builder-style setters =====

    /**
     * Sets the heading for this waypoint.
     *
     * @param heading Heading in radians
     * @return this for chaining
     */
    public Waypoint setHeading(double heading) {
        this.heading = heading;
        this.headingMode = HeadingMode.FIXED;
        return this;
    }

    /**
     * Sets the heading mode.
     *
     * @param mode The heading mode
     * @return this for chaining
     */
    public Waypoint setHeadingMode(HeadingMode mode) {
        this.headingMode = mode;
        return this;
    }

    /**
     * Sets a target point that the robot should face.
     *
     * @param target The point to face
     * @return this for chaining
     */
    public Waypoint setHeadingTarget(Translation2d target) {
        this.headingTarget = target;
        this.headingMode = HeadingMode.TARGET;
        return this;
    }

    /**
     * Sets the approach speed.
     *
     * @param speed Speed from 0 to 1
     * @return this for chaining
     */
    public Waypoint setSpeed(double speed) {
        this.speed = Math.max(0, Math.min(1, speed));
        return this;
    }

    /**
     * Sets the position tolerance.
     *
     * @param tolerance Position tolerance in mm
     * @return this for chaining
     */
    public Waypoint setPositionTolerance(double tolerance) {
        this.positionTolerance = tolerance;
        return this;
    }

    /**
     * Sets the heading tolerance.
     *
     * @param tolerance Heading tolerance in radians
     * @return this for chaining
     */
    public Waypoint setHeadingTolerance(double tolerance) {
        this.headingTolerance = tolerance;
        return this;
    }

    /**
     * Sets an action to perform when this waypoint is reached.
     *
     * @param action The action to run
     * @return this for chaining
     */
    public Waypoint setAction(Runnable action) {
        this.action = action;
        return this;
    }

    /**
     * Sets a delay before running the action.
     *
     * @param seconds Delay in seconds
     * @return this for chaining
     */
    public Waypoint setActionDelay(double seconds) {
        this.actionDelay = seconds;
        return this;
    }

    // ===== Getters =====

    public Translation2d getPosition() {
        return position;
    }

    public double getX() {
        return position.getX();
    }

    public double getY() {
        return position.getY();
    }

    /**
     * Gets the target heading based on the heading mode and current position.
     *
     * @param currentPose Current robot pose (used for PATH and TARGET modes)
     * @return Target heading in radians
     */
    public double getTargetHeading(Pose2d currentPose) {
        switch (headingMode) {
            case PATH:
                // Face direction of travel
                double dx = position.getX() - currentPose.getX();
                double dy = position.getY() - currentPose.getY();
                if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
                    return Math.atan2(dy, dx);
                }
                return currentPose.getHeading(); // At waypoint, maintain heading

            case TARGET:
                if (headingTarget != null) {
                    double tdx = headingTarget.getX() - currentPose.getX();
                    double tdy = headingTarget.getY() - currentPose.getY();
                    return Math.atan2(tdy, tdx);
                }
                return heading;

            case MAINTAIN:
                return currentPose.getHeading();

            case FIXED:
            default:
                return heading;
        }
    }

    public HeadingMode getHeadingMode() {
        return headingMode;
    }

    public double getSpeed() {
        return speed;
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public Runnable getAction() {
        return action;
    }

    public double getActionDelay() {
        return actionDelay;
    }

    /**
     * Checks if the given pose has reached this waypoint.
     *
     * @param currentPose The current pose to check
     * @return true if within tolerance
     */
    public boolean isReached(Pose2d currentPose) {
        double posError = currentPose.getTranslation().distanceTo(position);
        if (posError > positionTolerance) {
            return false;
        }

        // Check heading if mode requires it
        if (headingMode == HeadingMode.FIXED || headingMode == HeadingMode.TARGET) {
            double targetHeading = getTargetHeading(currentPose);
            double headingError = Math.abs(Pose2d.angleDifference(currentPose.getHeading(), targetHeading));
            return headingError <= headingTolerance;
        }

        return true;
    }

    /**
     * Converts to a Pose2d using the current pose for heading calculation.
     *
     * @param currentPose Current robot pose
     * @return Pose2d with position and calculated heading
     */
    public Pose2d toPose(Pose2d currentPose) {
        return new Pose2d(position, getTargetHeading(currentPose));
    }

    @Override
    public String toString() {
        return String.format("Waypoint(%.0f, %.0f, %s @ %.0f%%)",
                position.getX(), position.getY(), headingMode, speed * 100);
    }
}
