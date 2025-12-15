package org.firstinspires.ftc.teamcode.pickle.geometry;

/**
 * Represents a 2D translation (position) in field coordinates.
 *
 * <p>This is a simple immutable class representing an (x, y) position in millimeters.
 * Unlike WPILib's Translation2d, this is designed for FTC's needs with mm as the primary unit.
 *
 * <h3>Coordinate System Convention:</h3>
 * <ul>
 *   <li>Origin (0, 0) is at the center of the field</li>
 *   <li>+X points from Blue alliance wall toward Red alliance wall</li>
 *   <li>+Y points from audience side toward far side (consistent with team convention)</li>
 * </ul>
 */
public class Translation2d {
    private final double x;
    private final double y;

    /**
     * Constructs a Translation2d with the given x and y components.
     *
     * @param x The x component in millimeters
     * @param y The y component in millimeters
     */
    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a Translation2d at the origin (0, 0).
     */
    public Translation2d() {
        this(0.0, 0.0);
    }

    /**
     * @return The x component in millimeters
     */
    public double getX() {
        return x;
    }

    /**
     * @return The y component in millimeters
     */
    public double getY() {
        return y;
    }

    /**
     * @return The x component in inches
     */
    public double getXInches() {
        return x / 25.4;
    }

    /**
     * @return The y component in inches
     */
    public double getYInches() {
        return y / 25.4;
    }

    /**
     * Calculates the Euclidean distance from this translation to another.
     *
     * @param other The other translation
     * @return Distance in millimeters
     */
    public double distanceTo(Translation2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns the norm (magnitude) of this translation from the origin.
     *
     * @return Distance from origin in millimeters
     */
    public double getNorm() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Adds another translation to this one.
     *
     * @param other The translation to add
     * @return A new Translation2d representing the sum
     */
    public Translation2d plus(Translation2d other) {
        return new Translation2d(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another translation from this one.
     *
     * @param other The translation to subtract
     * @return A new Translation2d representing the difference
     */
    public Translation2d minus(Translation2d other) {
        return new Translation2d(this.x - other.x, this.y - other.y);
    }

    /**
     * Multiplies this translation by a scalar.
     *
     * @param scalar The scalar to multiply by
     * @return A new scaled Translation2d
     */
    public Translation2d times(double scalar) {
        return new Translation2d(this.x * scalar, this.y * scalar);
    }

    /**
     * Returns a new Translation2d with both components negated.
     * This is equivalent to rotating 180 degrees about the origin.
     *
     * @return The negated translation
     */
    public Translation2d unaryMinus() {
        return new Translation2d(-x, -y);
    }

    /**
     * Mirrors this translation for the opposite alliance.
     * For a field-centric coordinate system with origin at center,
     * this negates both x and y (180-degree rotation about origin).
     *
     * @return A new Translation2d mirrored for the opposite alliance
     */
    public Translation2d mirrorForBlue() {
        return new Translation2d(-x, -y);
    }

    /**
     * Creates a Translation2d from polar coordinates.
     *
     * @param distance Distance from origin in millimeters
     * @param angleRadians Angle in radians (0 = +X direction, counter-clockwise positive)
     * @return A new Translation2d
     */
    public static Translation2d fromPolar(double distance, double angleRadians) {
        return new Translation2d(
                distance * Math.cos(angleRadians),
                distance * Math.sin(angleRadians)
        );
    }

    @Override
    public String toString() {
        return String.format("Translation2d(x=%.1f mm, y=%.1f mm)", x, y);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Translation2d that = (Translation2d) obj;
        return Double.compare(that.x, x) == 0 && Double.compare(that.y, y) == 0;
    }

    @Override
    public int hashCode() {
        long xBits = Double.doubleToLongBits(x);
        long yBits = Double.doubleToLongBits(y);
        return (int) (xBits ^ (xBits >>> 32) ^ yBits ^ (yBits >>> 32));
    }
}
