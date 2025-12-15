package org.firstinspires.ftc.teamcode.pickle.field;

import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Rectangle2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

/**
 * Field element positions for the 2025-2026 DECODE season.
 *
 * <h2>DECODE Field Layout Overview</h2>
 * <p>The DECODE field includes:</p>
 * <ul>
 *   <li><b>Goals</b>: Scoring areas in the far corners (Red in top-right, Blue in top-left)</li>
 *   <li><b>Ramps</b>: Diagonal pathways leading up to goals</li>
 *   <li><b>Gates</b>: Entry points at the base of ramps (2.75" x 10")</li>
 *   <li><b>Bases</b>: Designated parking zones for end-game points (18" x 18")</li>
 *   <li><b>Loading Zones</b>: Areas where human players can load artifacts (23" x 23")</li>
 *   <li><b>Obelisks</b>: Central field elements with AprilTags 21, 22, 23 (not for navigation)</li>
 *   <li><b>Secret Tunnel</b>: Passage zone (46.5" x 6.125")</li>
 * </ul>
 *
 * <h2>Design Philosophy</h2>
 * <p>All constants are defined for the <b>RED alliance</b> first. Blue alliance positions
 * are derived through mirroring functions. This ensures symmetry and reduces configuration
 * errors.</p>
 *
 * <h2>Official Measurements</h2>
 * <p>Measurements are from the official DECODE Competition Manual (Section 9: ARENA).</p>
 * <ul>
 *   <li>Base Zone: 18" ± 0.125" (457.2 mm)</li>
 *   <li>Loading Zone: ~23" x 23" (584.2 mm)</li>
 *   <li>Gate Zone: 2.75" x 10" (69.85 mm x 254 mm)</li>
 *   <li>Goal Structure: ~27" x 27" x 54" tall</li>
 * </ul>
 *
 * @see FieldConstants for base field dimensions
 * @see Alliance for alliance-specific operations
 */
public final class DecodeField {
    private DecodeField() {
        // Utility class - no instantiation
    }

    // ========================================================================
    // APRILTAG IDS (from official DECODE Competition Manual)
    // ========================================================================

    /**
     * AprilTag ID on the Red alliance goal.
     * Use this for goal alignment; ignore Obelisk tags for navigation.
     */
    public static final int APRILTAG_RED_GOAL = 24;

    /**
     * AprilTag ID on the Blue alliance goal.
     */
    public static final int APRILTAG_BLUE_GOAL = 20;

    /**
     * AprilTag IDs on the Obelisk (central field element).
     * These are on the three rectangular faces and should NOT be used for localization.
     */
    public static final int APRILTAG_OBELISK_FACE_1 = 21;
    public static final int APRILTAG_OBELISK_FACE_2 = 22;
    public static final int APRILTAG_OBELISK_FACE_3 = 23;

    // ========================================================================
    // SECRET TUNNEL ZONE (Official: 46.5" x 6.125")
    // ========================================================================

    /**
     * Secret Tunnel length: 46.5" (1181.1 mm).
     */
    public static final double SECRET_TUNNEL_LENGTH_MM = FieldConstants.inchesToMm(46.5);

    /**
     * Secret Tunnel width: 6.125" (155.6 mm).
     */
    public static final double SECRET_TUNNEL_WIDTH_MM = FieldConstants.inchesToMm(6.125);

    // ========================================================================
    // GOAL POSITIONS
    // ========================================================================

    /**
     * Goal structure dimensions: approximately 27" x 27" x 54" tall.
     * The triangular opening is ~26.5" wide by 18.3" deep.
     * Top lip is 38.75" from tile surface.
     */
    public static final double GOAL_WIDTH_MM = FieldConstants.inchesToMm(27.0);
    public static final double GOAL_DEPTH_MM = FieldConstants.inchesToMm(27.0);
    public static final double GOAL_HEIGHT_MM = FieldConstants.inchesToMm(54.0);
    public static final double GOAL_OPENING_HEIGHT_MM = FieldConstants.inchesToMm(38.75);

    /**
     * Distance from the side wall (X-edge) to the goal center.
     * Goal is positioned in the corner, offset by half its width plus clearance.
     */
    public static final double GOAL_OFFSET_X_MM = GOAL_WIDTH_MM / 2.0 + FieldConstants.inchesToMm(3.0);

    /**
     * Distance from the far wall (Y-edge) to the goal center.
     */
    public static final double GOAL_OFFSET_Y_MM = GOAL_DEPTH_MM / 2.0 + FieldConstants.inchesToMm(3.0);

    /**
     * Red alliance goal center position.
     * Located in the top-right corner (positive X, positive Y).
     */
    public static final Translation2d RED_GOAL_CENTER = new Translation2d(
            FieldConstants.HALF_FIELD_MM - GOAL_OFFSET_X_MM,
            FieldConstants.HALF_FIELD_MM - GOAL_OFFSET_Y_MM
    );

    /**
     * Blue alliance goal center position.
     * Derived by mirroring the red goal about the origin.
     */
    public static final Translation2d BLUE_GOAL_CENTER = RED_GOAL_CENTER.mirrorForBlue();

    // ========================================================================
    // RAMP AND GATE POSITIONS
    // ========================================================================

    /**
     * Distance from goal center to where the ramp starts.
     * TODO: Measure from official DECODE field diagram.
     */
    public static final double RAMP_GOAL_OFFSET_MM = 200.0;

    /**
     * Length of the ramp from start to end (gate).
     * TODO: Measure from official DECODE field diagram.
     */
    public static final double RAMP_LENGTH_MM = 600.0;

    /**
     * Ramps are oriented at 45 degrees (diagonal from corner toward center).
     * This factor converts linear distance to X/Y components.
     */
    private static final double RAMP_DIAGONAL_FACTOR = 1.0 / Math.sqrt(2.0);

    /**
     * Red ramp start point (closer to goal).
     */
    public static final Translation2d RED_RAMP_START = new Translation2d(
            RED_GOAL_CENTER.getX() - RAMP_GOAL_OFFSET_MM * RAMP_DIAGONAL_FACTOR,
            RED_GOAL_CENTER.getY() - RAMP_GOAL_OFFSET_MM * RAMP_DIAGONAL_FACTOR
    );

    /**
     * Red ramp end point (at the gate, further from goal).
     */
    public static final Translation2d RED_RAMP_END = new Translation2d(
            RED_RAMP_START.getX() - RAMP_LENGTH_MM * RAMP_DIAGONAL_FACTOR,
            RED_RAMP_START.getY() - RAMP_LENGTH_MM * RAMP_DIAGONAL_FACTOR
    );

    /**
     * Blue ramp start point (closer to goal).
     */
    public static final Translation2d BLUE_RAMP_START = RED_RAMP_START.mirrorForBlue();

    /**
     * Blue ramp end point (at the gate).
     */
    public static final Translation2d BLUE_RAMP_END = RED_RAMP_END.mirrorForBlue();

    // ========================================================================
    // GATE REGIONS (Official: 2.75" wide x 10" long)
    // ========================================================================

    /**
     * Gate zone width: 2.75" (69.85 mm) - official measurement.
     * This is perpendicular to the ramp direction.
     */
    public static final double GATE_WIDTH_MM = FieldConstants.inchesToMm(2.75);

    /**
     * Gate zone length: 10" (254 mm) - official measurement.
     * This is along the ramp direction.
     */
    public static final double GATE_LENGTH_MM = FieldConstants.inchesToMm(10.0);

    /**
     * Half-width of the gate region (perpendicular to ramp direction).
     */
    public static final double GATE_HALF_WIDTH_MM = GATE_WIDTH_MM / 2.0;

    /**
     * Half-depth of the gate region (along ramp direction).
     */
    public static final double GATE_HALF_DEPTH_MM = GATE_LENGTH_MM / 2.0;

    /**
     * Red gate region (area around the ramp entrance).
     */
    public static final Rectangle2d RED_GATE_REGION = Rectangle2d.fromCenter(
            RED_RAMP_END,
            2.0 * GATE_HALF_WIDTH_MM,
            2.0 * GATE_HALF_DEPTH_MM
    );

    /**
     * Blue gate region.
     */
    public static final Rectangle2d BLUE_GATE_REGION = RED_GATE_REGION.mirrorForBlue();

    // ========================================================================
    // BASE (PARKING ZONE) POSITIONS
    // ========================================================================

    /**
     * Base parking zone side length (approximately 18 inches = 457.2 mm).
     * This is the size of the taped square on the field.
     */
    public static final double BASE_SIZE_MM = 18.0 * 25.4; // 457.2 mm

    /**
     * Distance from the side wall (X-edge) to the base center.
     * TODO: Measure from official DECODE field diagram.
     */
    public static final double BASE_OFFSET_X_MM = 600.0;

    /**
     * Distance from the far wall (Y-edge) to the base center.
     * TODO: Measure from official DECODE field diagram.
     */
    public static final double BASE_OFFSET_Y_MM = 300.0;

    /**
     * Red base center position.
     */
    public static final Translation2d RED_BASE_CENTER = new Translation2d(
            FieldConstants.HALF_FIELD_MM - BASE_OFFSET_X_MM,
            FieldConstants.HALF_FIELD_MM - BASE_OFFSET_Y_MM
    );

    /**
     * Blue base center position.
     */
    public static final Translation2d BLUE_BASE_CENTER = RED_BASE_CENTER.mirrorForBlue();

    /**
     * Red base parking region.
     */
    public static final Rectangle2d RED_BASE_REGION = Rectangle2d.squareFromCenter(
            RED_BASE_CENTER, BASE_SIZE_MM
    );

    /**
     * Blue base parking region.
     */
    public static final Rectangle2d BLUE_BASE_REGION = Rectangle2d.squareFromCenter(
            BLUE_BASE_CENTER, BASE_SIZE_MM
    );

    // ========================================================================
    // LOADING ZONE POSITIONS (Official: ~23" x 23")
    // ========================================================================

    /**
     * Loading zone size: approximately 23" x 23" (584.2 mm).
     * Bounded by white tape and field perimeters.
     */
    public static final double LOADING_ZONE_SIZE_MM = FieldConstants.inchesToMm(23.0);

    /**
     * Loading zone depth (distance into the field from the wall).
     */
    public static final double LOADING_ZONE_DEPTH_MM = LOADING_ZONE_SIZE_MM;

    /**
     * Loading zone width (along the wall).
     */
    public static final double LOADING_ZONE_WIDTH_MM = LOADING_ZONE_SIZE_MM;

    /**
     * Distance from center line (X=0) to the loading zone center.
     * Blue loading zone is on the left (-X), Red is on the right (+X).
     * Positioned in each alliance's audience-side corner area.
     */
    public static final double LOADING_ZONE_X_OFFSET_MM = FieldConstants.HALF_FIELD_MM - LOADING_ZONE_SIZE_MM / 2.0;

    /**
     * Blue loading zone center (near audience wall on blue side).
     */
    public static final Translation2d BLUE_LOADING_CENTER = new Translation2d(
            -LOADING_ZONE_X_OFFSET_MM,
            -FieldConstants.HALF_FIELD_MM + LOADING_ZONE_DEPTH_MM / 2.0
    );

    /**
     * Red loading zone center (near audience wall on red side).
     */
    public static final Translation2d RED_LOADING_CENTER = BLUE_LOADING_CENTER.mirrorForBlue();

    /**
     * Blue loading zone region.
     */
    public static final Rectangle2d BLUE_LOADING_REGION = Rectangle2d.fromCenter(
            BLUE_LOADING_CENTER,
            LOADING_ZONE_WIDTH_MM,
            LOADING_ZONE_DEPTH_MM
    );

    /**
     * Red loading zone region.
     */
    public static final Rectangle2d RED_LOADING_REGION = Rectangle2d.fromCenter(
            RED_LOADING_CENTER,
            LOADING_ZONE_WIDTH_MM,
            LOADING_ZONE_DEPTH_MM
    );

    // ========================================================================
    // STARTING POSITIONS
    // ========================================================================

    /**
     * Distance from the wall for starting positions.
     * Robot center is this far from the edge when placed in starting position.
     */
    public static final double START_WALL_OFFSET_MM = FieldConstants.inchesToMm(9.0);

    /**
     * Red starting position 1 (near audience corner).
     * Robot faces toward the goal (approximately 135 degrees from +X).
     */
    public static final Pose2d RED_START_1 = new Pose2d(
            FieldConstants.HALF_FIELD_MM - START_WALL_OFFSET_MM,
            -FieldConstants.HALF_FIELD_MM + START_WALL_OFFSET_MM,
            Math.toRadians(135.0)  // Facing toward center/goal
    );

    /**
     * Red starting position 2 (further from audience corner, on far wall).
     */
    public static final Pose2d RED_START_2 = new Pose2d(
            FieldConstants.HALF_FIELD_MM - START_WALL_OFFSET_MM,
            FieldConstants.HALF_FIELD_MM - FieldConstants.inchesToMm(24.0),
            Math.toRadians(180.0)  // Facing toward center
    );

    /**
     * Blue starting position 1 (near audience corner).
     */
    public static final Pose2d BLUE_START_1 = RED_START_1.mirrorForBlue();

    /**
     * Blue starting position 2 (further from audience corner).
     */
    public static final Pose2d BLUE_START_2 = RED_START_2.mirrorForBlue();

    // ========================================================================
    // ALLIANCE-AWARE ACCESSOR METHODS
    // ========================================================================

    /**
     * Gets the goal center for the specified alliance.
     *
     * @param alliance The alliance
     * @return Goal center position
     */
    public static Translation2d getGoalCenter(Alliance alliance) {
        return alliance == Alliance.RED ? RED_GOAL_CENTER : BLUE_GOAL_CENTER;
    }

    /**
     * Gets the base center for the specified alliance.
     *
     * @param alliance The alliance
     * @return Base center position
     */
    public static Translation2d getBaseCenter(Alliance alliance) {
        return alliance == Alliance.RED ? RED_BASE_CENTER : BLUE_BASE_CENTER;
    }

    /**
     * Gets the base parking region for the specified alliance.
     *
     * @param alliance The alliance
     * @return Base parking region
     */
    public static Rectangle2d getBaseRegion(Alliance alliance) {
        return alliance == Alliance.RED ? RED_BASE_REGION : BLUE_BASE_REGION;
    }

    /**
     * Gets the loading zone center for the specified alliance.
     *
     * @param alliance The alliance
     * @return Loading zone center position
     */
    public static Translation2d getLoadingCenter(Alliance alliance) {
        return alliance == Alliance.RED ? RED_LOADING_CENTER : BLUE_LOADING_CENTER;
    }

    /**
     * Gets the loading zone region for the specified alliance.
     *
     * @param alliance The alliance
     * @return Loading zone region
     */
    public static Rectangle2d getLoadingRegion(Alliance alliance) {
        return alliance == Alliance.RED ? RED_LOADING_REGION : BLUE_LOADING_REGION;
    }

    /**
     * Gets the ramp start point (near goal) for the specified alliance.
     *
     * @param alliance The alliance
     * @return Ramp start position
     */
    public static Translation2d getRampStart(Alliance alliance) {
        return alliance == Alliance.RED ? RED_RAMP_START : BLUE_RAMP_START;
    }

    /**
     * Gets the ramp end point (at gate) for the specified alliance.
     *
     * @param alliance The alliance
     * @return Ramp end position
     */
    public static Translation2d getRampEnd(Alliance alliance) {
        return alliance == Alliance.RED ? RED_RAMP_END : BLUE_RAMP_END;
    }

    /**
     * Gets the gate region for the specified alliance.
     *
     * @param alliance The alliance
     * @return Gate region
     */
    public static Rectangle2d getGateRegion(Alliance alliance) {
        return alliance == Alliance.RED ? RED_GATE_REGION : BLUE_GATE_REGION;
    }

    /**
     * Gets the AprilTag ID for the specified alliance's goal.
     * Use this to filter AprilTag detections for goal alignment.
     *
     * @param alliance The alliance
     * @return Goal AprilTag ID (24 for Red, 20 for Blue)
     */
    public static int getGoalAprilTagId(Alliance alliance) {
        return alliance == Alliance.RED ? APRILTAG_RED_GOAL : APRILTAG_BLUE_GOAL;
    }

    /**
     * Checks if an AprilTag ID belongs to the Obelisk.
     * Obelisk tags should NOT be used for robot localization.
     *
     * @param tagId The AprilTag ID to check
     * @return true if this is an Obelisk tag (21, 22, or 23)
     */
    public static boolean isObeliskTag(int tagId) {
        return tagId == APRILTAG_OBELISK_FACE_1 ||
               tagId == APRILTAG_OBELISK_FACE_2 ||
               tagId == APRILTAG_OBELISK_FACE_3;
    }

    /**
     * Checks if an AprilTag ID is a goal tag.
     *
     * @param tagId The AprilTag ID to check
     * @return true if this is a goal tag (20 or 24)
     */
    public static boolean isGoalTag(int tagId) {
        return tagId == APRILTAG_RED_GOAL || tagId == APRILTAG_BLUE_GOAL;
    }

    /**
     * Gets the starting pose for the specified alliance and position number.
     *
     * @param alliance The alliance
     * @param position Position number (1 or 2)
     * @return Starting pose
     * @throws IllegalArgumentException if position is not 1 or 2
     */
    public static Pose2d getStartingPose(Alliance alliance, int position) {
        if (position == 1) {
            return alliance == Alliance.RED ? RED_START_1 : BLUE_START_1;
        } else if (position == 2) {
            return alliance == Alliance.RED ? RED_START_2 : BLUE_START_2;
        } else {
            throw new IllegalArgumentException("Position must be 1 or 2, got: " + position);
        }
    }

    // ========================================================================
    // COORDINATE TRANSFORMATION HELPERS
    // ========================================================================

    /**
     * Transforms a pose from "alliance-local" coordinates to field coordinates.
     *
     * <p>Alliance-local coordinates assume:</p>
     * <ul>
     *   <li>Origin is at the alliance's base center</li>
     *   <li>+X points toward the alliance's goal</li>
     *   <li>+Y points "left" when facing the goal</li>
     * </ul>
     *
     * @param localPose Pose in alliance-local coordinates
     * @param alliance The alliance
     * @return Pose in field coordinates
     */
    public static Pose2d allianceLocalToField(Pose2d localPose, Alliance alliance) {
        Translation2d baseCenter = getBaseCenter(alliance);

        if (alliance == Alliance.RED) {
            // Red: +X (local) maps to -X,-Y direction (toward goal in top-right)
            // This is a 135-degree rotation
            double cos135 = -Math.sqrt(2.0) / 2.0;
            double sin135 = Math.sqrt(2.0) / 2.0;

            double fieldX = baseCenter.getX() + localPose.getX() * cos135 - localPose.getY() * sin135;
            double fieldY = baseCenter.getY() + localPose.getX() * sin135 + localPose.getY() * cos135;
            double fieldHeading = localPose.getHeading() + Math.toRadians(135.0);

            return new Pose2d(fieldX, fieldY, fieldHeading);
        } else {
            // Blue: mirror of red transformation
            // This is a -45-degree rotation (or 315 degrees)
            double cos315 = Math.sqrt(2.0) / 2.0;
            double sin315 = -Math.sqrt(2.0) / 2.0;

            double fieldX = baseCenter.getX() + localPose.getX() * cos315 - localPose.getY() * sin315;
            double fieldY = baseCenter.getY() + localPose.getX() * sin315 + localPose.getY() * cos315;
            double fieldHeading = localPose.getHeading() + Math.toRadians(-45.0);

            return new Pose2d(fieldX, fieldY, fieldHeading);
        }
    }

    /**
     * Mirrors a pose defined for Red alliance to the equivalent Blue alliance pose.
     *
     * @param redPose Pose defined for Red alliance
     * @return Equivalent pose for Blue alliance
     */
    public static Pose2d mirrorForBlue(Pose2d redPose) {
        return redPose.mirrorForBlue();
    }

    /**
     * Mirrors a translation defined for Red alliance to the equivalent Blue alliance position.
     *
     * @param redPoint Point defined for Red alliance
     * @return Equivalent point for Blue alliance
     */
    public static Translation2d mirrorForBlue(Translation2d redPoint) {
        return redPoint.mirrorForBlue();
    }

    // ========================================================================
    // NAVIGATION HELPERS
    // ========================================================================

    /**
     * Checks if a pose's position is within the specified alliance's base region.
     *
     * @param pose The pose to check
     * @param alliance The alliance
     * @return true if the robot is parked in the base
     */
    public static boolean isInBase(Pose2d pose, Alliance alliance) {
        return getBaseRegion(alliance).contains(pose);
    }

    /**
     * Checks if a pose's position is within the specified alliance's loading zone.
     *
     * @param pose The pose to check
     * @param alliance The alliance
     * @return true if the robot is in the loading zone
     */
    public static boolean isInLoadingZone(Pose2d pose, Alliance alliance) {
        return getLoadingRegion(alliance).contains(pose);
    }

    /**
     * Calculates the distance from a position to the goal center.
     *
     * @param position Current position
     * @param alliance The alliance (determines which goal)
     * @return Distance to goal in millimeters
     */
    public static double distanceToGoal(Translation2d position, Alliance alliance) {
        return position.distanceTo(getGoalCenter(alliance));
    }

    /**
     * Calculates the heading angle from a position to the goal center.
     *
     * @param position Current position
     * @param alliance The alliance (determines which goal)
     * @return Heading in radians that would point directly at the goal
     */
    public static double headingToGoal(Translation2d position, Alliance alliance) {
        Translation2d goal = getGoalCenter(alliance);
        double dx = goal.getX() - position.getX();
        double dy = goal.getY() - position.getY();
        return Math.atan2(dy, dx);
    }
}
