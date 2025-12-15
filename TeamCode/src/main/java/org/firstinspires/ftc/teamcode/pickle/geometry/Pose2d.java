package org.firstinspires.ftc.teamcode.pickle.geometry;

/**
 * Represents a 2D robot pose (position + heading) in field coordinates.
 *
 * <p>This is an immutable class representing (x, y, heading) where:
 * <ul>
 *   <li>x, y are in millimeters</li>
 *   <li>heading is in radians, where 0 = facing +X, counter-clockwise positive</li>
 * </ul>
 *
 * <h3>Heading Convention:</h3>
 * <ul>
 *   <li>0 radians = robot faces toward Red alliance (positive X)</li>
 *   <li>PI/2 radians (90 deg) = robot faces toward far side (positive Y)</li>
 *   <li>PI radians (180 deg) = robot faces toward Blue alliance (negative X)</li>
 *   <li>-PI/2 radians (-90 deg) = robot faces toward audience (negative Y)</li>
 * </ul>
 */
public class Pose2d {
    private final Translation2d translation;
    private final double heading; // radians

    /**
     * Constructs a Pose2d with the given position and heading.
     *
     * @param x X position in millimeters
     * @param y Y position in millimeters
     * @param headingRadians Heading in radians (0 = +X, CCW positive)
     */
    public Pose2d(double x, double y, double headingRadians) {
        this.translation = new Translation2d(x, y);
        this.heading = normalizeAngle(headingRadians);
    }

    /**
     * Constructs a Pose2d with the given translation and heading.
     *
     * @param translation The position
     * @param headingRadians Heading in radians
     */
    public Pose2d(Translation2d translation, double headingRadians) {
        this.translation = translation;
        this.heading = normalizeAngle(headingRadians);
    }

    /**
     * Constructs a Pose2d at the origin facing +X.
     */
    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * @return The X position in millimeters
     */
    public double getX() {
        return translation.getX();
    }

    /**
     * @return The Y position in millimeters
     */
    public double getY() {
        return translation.getY();
    }

    /**
     * @return The heading in radians, normalized to [-PI, PI]
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return The heading in degrees, normalized to [-180, 180]
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    /**
     * @return The translation component of this pose
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Returns a new Pose2d with the same position but a different heading.
     *
     * @param newHeadingRadians The new heading in radians
     * @return A new Pose2d
     */
    public Pose2d withHeading(double newHeadingRadians) {
        return new Pose2d(translation, newHeadingRadians);
    }

    /**
     * Returns a new Pose2d with a relative translation applied.
     * The translation is applied in field coordinates (not robot-relative).
     *
     * @param delta The translation to add
     * @return A new Pose2d
     */
    public Pose2d plus(Translation2d delta) {
        return new Pose2d(translation.plus(delta), heading);
    }

    /**
     * Calculates the distance from this pose to another pose (ignoring heading).
     *
     * @param other The other pose
     * @return Distance in millimeters
     */
    public double distanceTo(Pose2d other) {
        return translation.distanceTo(other.translation);
    }

    /**
     * Mirrors this pose for the opposite alliance.
     * <ul>
     *   <li>Position is mirrored by negating both x and y</li>
     *   <li>Heading is rotated by 180 degrees</li>
     * </ul>
     *
     * @return A new Pose2d mirrored for Blue alliance (if this was Red)
     */
    public Pose2d mirrorForBlue() {
        return new Pose2d(
                -translation.getX(),
                -translation.getY(),
                normalizeAngle(heading + Math.PI)
        );
    }

    /**
     * Normalizes an angle to the range [-PI, PI].
     *
     * @param angleRadians The angle to normalize
     * @return The normalized angle
     */
    public static double normalizeAngle(double angleRadians) {
        double result = angleRadians;
        while (result > Math.PI) result -= 2 * Math.PI;
        while (result < -Math.PI) result += 2 * Math.PI;
        return result;
    }

    /**
     * Calculates the shortest angular difference between two headings.
     *
     * @param from Starting heading in radians
     * @param to Target heading in radians
     * @return The shortest angular distance in radians (can be negative)
     */
    public static double angleDifference(double from, double to) {
        double diff = normalizeAngle(to - from);
        return diff;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.1f mm, y=%.1f mm, heading=%.1f deg)",
                getX(), getY(), getHeadingDegrees());
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Pose2d pose2d = (Pose2d) obj;
        return translation.equals(pose2d.translation) &&
                Double.compare(pose2d.heading, heading) == 0;
    }

    @Override
    public int hashCode() {
        int result = translation.hashCode();
        long headingBits = Double.doubleToLongBits(heading);
        result = 31 * result + (int) (headingBits ^ (headingBits >>> 32));
        return result;
    }
}
