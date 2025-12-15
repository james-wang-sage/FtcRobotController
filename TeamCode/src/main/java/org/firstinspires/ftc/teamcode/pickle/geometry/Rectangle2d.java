package org.firstinspires.ftc.teamcode.pickle.geometry;

/**
 * Represents an axis-aligned bounding box (AABB) in 2D field coordinates.
 *
 * <p>This is useful for defining regions like the Base parking zone, Loading Zone,
 * Gate area, or other rectangular field elements.
 *
 * <p>The rectangle is defined by its minimum corner (minX, minY) and dimensions (width, height).
 */
public class Rectangle2d {
    private final double minX;
    private final double minY;
    private final double width;
    private final double height;

    /**
     * Constructs a Rectangle2d from the minimum corner and dimensions.
     *
     * @param minX The minimum X coordinate (left edge) in mm
     * @param minY The minimum Y coordinate (bottom edge) in mm
     * @param width The width in mm (must be positive)
     * @param height The height in mm (must be positive)
     */
    public Rectangle2d(double minX, double minY, double width, double height) {
        if (width < 0 || height < 0) {
            throw new IllegalArgumentException("Width and height must be non-negative");
        }
        this.minX = minX;
        this.minY = minY;
        this.width = width;
        this.height = height;
    }

    /**
     * Constructs a Rectangle2d centered at a given point with specified dimensions.
     *
     * @param center The center point
     * @param width The width in mm
     * @param height The height in mm
     * @return A new Rectangle2d centered at the given point
     */
    public static Rectangle2d fromCenter(Translation2d center, double width, double height) {
        return new Rectangle2d(
                center.getX() - width / 2.0,
                center.getY() - height / 2.0,
                width,
                height
        );
    }

    /**
     * Constructs a square Rectangle2d centered at a given point.
     *
     * @param center The center point
     * @param size The side length in mm
     * @return A new square Rectangle2d centered at the given point
     */
    public static Rectangle2d squareFromCenter(Translation2d center, double size) {
        return fromCenter(center, size, size);
    }

    /**
     * @return The minimum X coordinate (left edge)
     */
    public double getMinX() {
        return minX;
    }

    /**
     * @return The minimum Y coordinate (bottom edge)
     */
    public double getMinY() {
        return minY;
    }

    /**
     * @return The maximum X coordinate (right edge)
     */
    public double getMaxX() {
        return minX + width;
    }

    /**
     * @return The maximum Y coordinate (top edge)
     */
    public double getMaxY() {
        return minY + height;
    }

    /**
     * @return The width of the rectangle
     */
    public double getWidth() {
        return width;
    }

    /**
     * @return The height of the rectangle
     */
    public double getHeight() {
        return height;
    }

    /**
     * @return The center point of the rectangle
     */
    public Translation2d getCenter() {
        return new Translation2d(minX + width / 2.0, minY + height / 2.0);
    }

    /**
     * Checks if a point is contained within this rectangle.
     *
     * @param point The point to check
     * @return true if the point is inside or on the boundary of the rectangle
     */
    public boolean contains(Translation2d point) {
        return point.getX() >= minX && point.getX() <= getMaxX() &&
                point.getY() >= minY && point.getY() <= getMaxY();
    }

    /**
     * Checks if a pose's position is contained within this rectangle.
     * (Heading is ignored - only the translation is checked.)
     *
     * @param pose The pose to check
     * @return true if the pose's position is inside or on the boundary
     */
    public boolean contains(Pose2d pose) {
        return contains(pose.getTranslation());
    }

    /**
     * Checks if this rectangle overlaps with another rectangle.
     *
     * @param other The other rectangle
     * @return true if the rectangles overlap (including touching edges)
     */
    public boolean overlaps(Rectangle2d other) {
        return !(getMaxX() < other.minX || other.getMaxX() < minX ||
                getMaxY() < other.minY || other.getMaxY() < minY);
    }

    /**
     * Returns a new rectangle expanded by the given margin in all directions.
     *
     * @param margin The margin to add (can be negative to shrink)
     * @return A new expanded Rectangle2d
     */
    public Rectangle2d expand(double margin) {
        return new Rectangle2d(
                minX - margin,
                minY - margin,
                width + 2 * margin,
                height + 2 * margin
        );
    }

    /**
     * Mirrors this rectangle for the opposite alliance.
     * Both the center point and dimensions are mirrored about the origin.
     *
     * @return A new Rectangle2d mirrored for Blue alliance
     */
    public Rectangle2d mirrorForBlue() {
        // Mirror the center, keep same dimensions
        Translation2d center = getCenter();
        Translation2d mirroredCenter = center.mirrorForBlue();
        return fromCenter(mirroredCenter, width, height);
    }

    /**
     * Calculates the shortest distance from a point to this rectangle's boundary.
     * Returns 0 if the point is inside the rectangle.
     *
     * @param point The point to measure from
     * @return Distance in mm (0 if inside)
     */
    public double distanceToPoint(Translation2d point) {
        if (contains(point)) {
            return 0.0;
        }

        // Clamp point to rectangle bounds
        double closestX = Math.max(minX, Math.min(point.getX(), getMaxX()));
        double closestY = Math.max(minY, Math.min(point.getY(), getMaxY()));

        double dx = point.getX() - closestX;
        double dy = point.getY() - closestY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return String.format("Rectangle2d(x=[%.1f, %.1f], y=[%.1f, %.1f] mm)",
                minX, getMaxX(), minY, getMaxY());
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Rectangle2d that = (Rectangle2d) obj;
        return Double.compare(that.minX, minX) == 0 &&
                Double.compare(that.minY, minY) == 0 &&
                Double.compare(that.width, width) == 0 &&
                Double.compare(that.height, height) == 0;
    }

    @Override
    public int hashCode() {
        long result = Double.doubleToLongBits(minX);
        result = 31 * result + Double.doubleToLongBits(minY);
        result = 31 * result + Double.doubleToLongBits(width);
        result = 31 * result + Double.doubleToLongBits(height);
        return (int) (result ^ (result >>> 32));
    }
}
