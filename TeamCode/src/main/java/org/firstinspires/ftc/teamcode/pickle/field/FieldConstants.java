package org.firstinspires.ftc.teamcode.pickle.field;

import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Fundamental field constants for FTC competition fields.
 *
 * <h2>Coordinate System Convention</h2>
 * <p>We use a field-centric coordinate system with the geometric center as origin.
 * This makes symmetric operations and alliance mirroring straightforward.</p>
 *
 * <pre>
 *                    +Y (far side / away from audience)
 *                         ^
 *                         |
 *    BLUE_FAR_CORNER      |      RED_FAR_CORNER
 *    (-1828.8, 1828.8)    |      (1828.8, 1828.8)
 *         +---------------+---------------+
 *         |               |               |
 *         |    BLUE       |     RED       |
 *         |   ALLIANCE    |   ALLIANCE    |
 *  -X <---+----------- (0,0) -------------+---> +X (toward Red alliance)
 *         |               |               |
 *         |               |               |
 *         |               |               |
 *         +---------------+---------------+
 *    BLUE_AUDIENCE_CORNER |      RED_AUDIENCE_CORNER
 *    (-1828.8, -1828.8)   |      (1828.8, -1828.8)
 *                         |
 *                         v
 *                    -Y (audience side)
 * </pre>
 *
 * <h2>Unit Convention</h2>
 * <p>All measurements are in <b>millimeters</b> internally.
 * Helper methods are provided for inch conversions where needed.</p>
 *
 * <h2>Angle Convention</h2>
 * <ul>
 *   <li>0 radians = facing +X (toward Red alliance)</li>
 *   <li>Counter-clockwise is positive</li>
 * </ul>
 */
public final class FieldConstants {
    private FieldConstants() {
        // Utility class - no instantiation
    }

    // ========================================================================
    // FIELD DIMENSIONS
    // ========================================================================

    /**
     * Standard FTC field size: 12 feet x 12 feet.
     */
    public static final double FIELD_SIZE_FEET = 12.0;

    /**
     * Field size in inches (144 inches).
     */
    public static final double FIELD_SIZE_INCHES = FIELD_SIZE_FEET * 12.0;

    /**
     * Field size in millimeters (3657.6 mm).
     */
    public static final double FIELD_SIZE_MM = FIELD_SIZE_INCHES * 25.4;

    /**
     * Half field size in millimeters (1828.8 mm).
     * Distance from center to any edge.
     */
    public static final double HALF_FIELD_MM = FIELD_SIZE_MM / 2.0;

    /**
     * Half field size in inches (72 inches).
     */
    public static final double HALF_FIELD_INCHES = FIELD_SIZE_INCHES / 2.0;

    // ========================================================================
    // TILE DIMENSIONS (standard foam tiles)
    // ========================================================================

    /**
     * Standard FTC tile size: 24 inches (2 feet).
     */
    public static final double TILE_SIZE_INCHES = 24.0;

    /**
     * Tile size in millimeters.
     */
    public static final double TILE_SIZE_MM = TILE_SIZE_INCHES * 25.4;

    /**
     * Number of tiles per side (6 tiles = 12 feet).
     */
    public static final int TILES_PER_SIDE = 6;

    // ========================================================================
    // FIELD CORNERS
    // ========================================================================

    /**
     * Blue alliance corner nearest the audience.
     * Bottom-left in the standard diagram.
     */
    public static final Translation2d BLUE_AUDIENCE_CORNER =
            new Translation2d(-HALF_FIELD_MM, -HALF_FIELD_MM);

    /**
     * Red alliance corner nearest the audience.
     * Bottom-right in the standard diagram.
     */
    public static final Translation2d RED_AUDIENCE_CORNER =
            new Translation2d(HALF_FIELD_MM, -HALF_FIELD_MM);

    /**
     * Blue alliance corner on the far side (away from audience).
     * Top-left in the standard diagram.
     */
    public static final Translation2d BLUE_FAR_CORNER =
            new Translation2d(-HALF_FIELD_MM, HALF_FIELD_MM);

    /**
     * Red alliance corner on the far side (away from audience).
     * Top-right in the standard diagram.
     */
    public static final Translation2d RED_FAR_CORNER =
            new Translation2d(HALF_FIELD_MM, HALF_FIELD_MM);

    // ========================================================================
    // UNIT CONVERSION HELPERS
    // ========================================================================

    /**
     * Converts inches to millimeters.
     *
     * @param inches Value in inches
     * @return Value in millimeters
     */
    public static double inchesToMm(double inches) {
        return inches * 25.4;
    }

    /**
     * Converts millimeters to inches.
     *
     * @param mm Value in millimeters
     * @return Value in inches
     */
    public static double mmToInches(double mm) {
        return mm / 25.4;
    }

    /**
     * Converts feet to millimeters.
     *
     * @param feet Value in feet
     * @return Value in millimeters
     */
    public static double feetToMm(double feet) {
        return feet * 12.0 * 25.4;
    }

    /**
     * Converts tile coordinates to millimeters.
     * Tile (0, 0) is at field center.
     *
     * @param tiles Number of tiles from center
     * @return Distance in millimeters
     */
    public static double tilesToMm(double tiles) {
        return tiles * TILE_SIZE_MM;
    }

    // ========================================================================
    // COORDINATE SYSTEM HELPERS
    // ========================================================================

    /**
     * Creates a Translation2d from tile coordinates.
     * Tile (0, 0) is at field center.
     *
     * @param tileX X position in tiles (positive = toward Red)
     * @param tileY Y position in tiles (positive = toward far side)
     * @return Translation2d in millimeters
     */
    public static Translation2d fromTiles(double tileX, double tileY) {
        return new Translation2d(tilesToMm(tileX), tilesToMm(tileY));
    }

    /**
     * Creates a Pose2d from tile coordinates and heading.
     *
     * @param tileX X position in tiles
     * @param tileY Y position in tiles
     * @param headingDegrees Heading in degrees (0 = toward Red, CCW positive)
     * @return Pose2d in millimeters with heading in radians
     */
    public static Pose2d poseFromTiles(double tileX, double tileY, double headingDegrees) {
        return new Pose2d(
                tilesToMm(tileX),
                tilesToMm(tileY),
                Math.toRadians(headingDegrees)
        );
    }

    /**
     * Checks if a point is within the legal field boundaries.
     *
     * @param point The point to check
     * @return true if the point is within the field
     */
    public static boolean isOnField(Translation2d point) {
        return Math.abs(point.getX()) <= HALF_FIELD_MM &&
                Math.abs(point.getY()) <= HALF_FIELD_MM;
    }
}
