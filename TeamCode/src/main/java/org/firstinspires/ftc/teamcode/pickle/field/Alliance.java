package org.firstinspires.ftc.teamcode.pickle.field;

/**
 * Represents the alliance color in FTC competition.
 *
 * <p>This enum is used throughout the codebase to handle alliance-specific
 * logic such as:
 * <ul>
 *   <li>Mirroring autonomous paths for Blue vs Red starting positions</li>
 *   <li>Selecting alliance-appropriate AprilTags for navigation</li>
 *   <li>Choosing correct goal positions for scoring</li>
 * </ul>
 *
 * <h3>Usage Example:</h3>
 * <pre>
 * // Get alliance-specific starting pose
 * Pose2d startPose = DecodeField.getStartingPose(Alliance.RED, 1);
 *
 * // Get goal position for current alliance
 * Translation2d goal = DecodeField.getGoalCenter(alliance);
 * </pre>
 */
public enum Alliance {
    /**
     * Red alliance - positioned on the +X side of the field.
     */
    RED,

    /**
     * Blue alliance - positioned on the -X side of the field.
     */
    BLUE;

    /**
     * Returns the opposite alliance.
     *
     * @return BLUE if this is RED, RED if this is BLUE
     */
    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }

    /**
     * Returns the X-direction sign for this alliance.
     * Red is on +X side, Blue is on -X side.
     *
     * @return +1 for RED, -1 for BLUE
     */
    public int xSign() {
        return this == RED ? 1 : -1;
    }

    /**
     * Returns the Y-direction sign for this alliance's "home" side.
     * Using the convention that each alliance's goal is in the far corner,
     * both alliances have goals on +Y, so this returns the appropriate sign
     * for approaching from the audience side.
     *
     * @return +1 (both alliances approach goals from -Y toward +Y)
     */
    public int ySign() {
        // Both alliances start near -Y and move toward +Y to score
        return 1;
    }
}
