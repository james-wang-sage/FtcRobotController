package org.firstinspires.ftc.teamcode.pickle.path;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pickle.drive.MecanumDriveHelper;
import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Road Runner-style path follower for mecanum drivetrains.
 *
 * <h2>What is Path Following?</h2>
 * <p>Path following allows the robot to smoothly navigate through a series of waypoints,
 * managing speed, heading, and position control automatically. This is inspired by
 * the Road Runner library used by many FTC teams.</p>
 *
 * <h2>Key Features</h2>
 * <ul>
 *   <li>Sequential waypoint navigation</li>
 *   <li>Multiple heading modes (fixed, path-following, target-facing)</li>
 *   <li>Configurable tolerances per waypoint</li>
 *   <li>Action callbacks at waypoints</li>
 *   <li>Smooth speed transitions</li>
 * </ul>
 *
 * <h2>Path Building</h2>
 * <pre>
 * PathFollower follower = new PathFollower(driveHelper);
 *
 * // Build a path using the fluent API
 * follower.buildPath()
 *     .forward(1000)                    // 1000mm forward
 *     .strafeRight(500)                 // 500mm right
 *     .turnTo(Math.toRadians(90))       // Turn to face 90°
 *     .splineTo(x, y, heading)          // Smooth curve to point
 *     .addWaypoint(new Waypoint(x, y).setHeadingMode(HeadingMode.TARGET)
 *                                      .setHeadingTarget(goalPosition))
 *     .build();
 * </pre>
 *
 * <h2>Following a Path</h2>
 * <pre>
 * // In loop()
 * if (!follower.isComplete()) {
 *     follower.update(odometry.getPose());
 * }
 * </pre>
 *
 * <h2>Comparison to Road Runner</h2>
 * <p>This is a simplified version of Road Runner's trajectory system. Key differences:</p>
 * <ul>
 *   <li>Uses waypoints instead of continuous trajectories</li>
 *   <li>No motion profiling (constant speed per segment)</li>
 *   <li>Simpler implementation suitable for beginners</li>
 * </ul>
 */
public class PathFollower {

    private final MecanumDriveHelper driveHelper;

    // Path state
    private List<Waypoint> waypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;
    private boolean pathComplete = false;
    private boolean running = false;

    // Timing
    private ElapsedTime waypointTimer = new ElapsedTime();
    private ElapsedTime pathTimer = new ElapsedTime();

    // Settings
    private double defaultSpeed = 0.5;
    private double defaultPositionTolerance = 50; // mm
    private double defaultHeadingTolerance = Math.toRadians(5);
    private double timeoutPerWaypoint = 5.0; // seconds

    // State tracking
    private boolean waitingForAction = false;
    private double actionWaitTime = 0;

    /**
     * Creates a new PathFollower.
     *
     * @param driveHelper The mecanum drive helper for motor control
     */
    public PathFollower(MecanumDriveHelper driveHelper) {
        this.driveHelper = driveHelper;
    }

    /**
     * Sets the default speed for waypoints that don't specify one.
     *
     * @param speed Speed from 0 to 1
     */
    public void setDefaultSpeed(double speed) {
        this.defaultSpeed = Math.max(0, Math.min(1, speed));
    }

    /**
     * Sets the timeout per waypoint before moving on.
     *
     * @param seconds Timeout in seconds
     */
    public void setWaypointTimeout(double seconds) {
        this.timeoutPerWaypoint = seconds;
    }

    /**
     * Starts building a new path from the current position.
     *
     * @return A PathBuilder for fluent path construction
     */
    public PathBuilder buildPath() {
        return new PathBuilder(this);
    }

    /**
     * Sets the path to follow.
     *
     * @param waypoints List of waypoints
     */
    public void setPath(List<Waypoint> waypoints) {
        this.waypoints = new ArrayList<>(waypoints);
        this.currentWaypointIndex = 0;
        this.pathComplete = false;
        this.running = true;
        this.waitingForAction = false;
        waypointTimer.reset();
        pathTimer.reset();
    }

    /**
     * Clears the current path.
     */
    public void clearPath() {
        waypoints.clear();
        currentWaypointIndex = 0;
        pathComplete = true;
        running = false;
    }

    /**
     * Updates the path follower. Call this every loop iteration.
     *
     * @param currentPose The robot's current pose from odometry
     */
    public void update(Pose2d currentPose) {
        if (!running || pathComplete || waypoints.isEmpty()) {
            driveHelper.stop();
            return;
        }

        Waypoint currentWaypoint = waypoints.get(currentWaypointIndex);

        // Check if we're waiting for an action delay
        if (waitingForAction) {
            driveHelper.stop();
            if (waypointTimer.seconds() >= actionWaitTime) {
                // Execute action if present
                if (currentWaypoint.getAction() != null) {
                    currentWaypoint.getAction().run();
                }
                waitingForAction = false;
                advanceToNextWaypoint();
            }
            return;
        }

        // Check if waypoint is reached
        if (currentWaypoint.isReached(currentPose) || waypointTimer.seconds() > timeoutPerWaypoint) {
            // Start action delay if needed
            if (currentWaypoint.getAction() != null && currentWaypoint.getActionDelay() > 0) {
                waitingForAction = true;
                actionWaitTime = currentWaypoint.getActionDelay();
                waypointTimer.reset();
                driveHelper.stop();
                return;
            }

            // Execute action immediately if no delay
            if (currentWaypoint.getAction() != null) {
                currentWaypoint.getAction().run();
            }

            advanceToNextWaypoint();
            return;
        }

        // Drive toward current waypoint
        Pose2d targetPose = currentWaypoint.toPose(currentPose);
        driveHelper.driveToPose(targetPose, currentPose, currentWaypoint.getSpeed());
    }

    private void advanceToNextWaypoint() {
        currentWaypointIndex++;
        if (currentWaypointIndex >= waypoints.size()) {
            pathComplete = true;
            running = false;
            driveHelper.stop();
        } else {
            waypointTimer.reset();
        }
    }

    /**
     * Pauses path following.
     */
    public void pause() {
        running = false;
        driveHelper.stop();
    }

    /**
     * Resumes path following.
     */
    public void resume() {
        if (!pathComplete && !waypoints.isEmpty()) {
            running = true;
        }
    }

    /**
     * Stops and clears the path.
     */
    public void stop() {
        clearPath();
        driveHelper.stop();
    }

    /**
     * Checks if the path is complete.
     *
     * @return true if all waypoints have been reached
     */
    public boolean isComplete() {
        return pathComplete;
    }

    /**
     * Checks if the path follower is currently running.
     *
     * @return true if following a path
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Gets the current waypoint index.
     *
     * @return Index of current waypoint (0-based)
     */
    public int getCurrentWaypointIndex() {
        return currentWaypointIndex;
    }

    /**
     * Gets the total number of waypoints.
     *
     * @return Number of waypoints in path
     */
    public int getWaypointCount() {
        return waypoints.size();
    }

    /**
     * Gets the current target waypoint.
     *
     * @return Current waypoint, or null if path complete
     */
    public Waypoint getCurrentWaypoint() {
        if (currentWaypointIndex < waypoints.size()) {
            return waypoints.get(currentWaypointIndex);
        }
        return null;
    }

    /**
     * Gets elapsed time since path started.
     *
     * @return Elapsed seconds
     */
    public double getElapsedTime() {
        return pathTimer.seconds();
    }

    /**
     * Gets progress as a percentage.
     *
     * @return Progress from 0 to 1
     */
    public double getProgress() {
        if (waypoints.isEmpty()) return 1.0;
        return (double) currentWaypointIndex / waypoints.size();
    }

    /**
     * Gets diagnostic information.
     *
     * @return Formatted diagnostic string
     */
    public String getDiagnostics() {
        if (pathComplete) {
            return String.format("Path complete (%.1fs)", pathTimer.seconds());
        }
        if (waypoints.isEmpty()) {
            return "No path set";
        }
        Waypoint wp = getCurrentWaypoint();
        return String.format("Waypoint %d/%d: %s (%.1fs)",
                currentWaypointIndex + 1,
                waypoints.size(),
                wp != null ? wp.toString() : "null",
                waypointTimer.seconds());
    }

    // ===== Path Builder Inner Class =====

    /**
     * Fluent builder for creating paths.
     */
    public static class PathBuilder {
        private final PathFollower follower;
        private final List<Waypoint> waypoints = new ArrayList<>();
        private Pose2d currentPose = new Pose2d(0, 0, 0);

        private double currentSpeed;
        private double currentPositionTolerance;
        private double currentHeadingTolerance;

        PathBuilder(PathFollower follower) {
            this.follower = follower;
            this.currentSpeed = follower.defaultSpeed;
            this.currentPositionTolerance = follower.defaultPositionTolerance;
            this.currentHeadingTolerance = follower.defaultHeadingTolerance;
        }

        /**
         * Sets the starting pose for relative movements.
         *
         * @param pose Starting pose
         * @return this for chaining
         */
        public PathBuilder from(Pose2d pose) {
            this.currentPose = pose;
            return this;
        }

        /**
         * Sets the speed for subsequent waypoints.
         *
         * @param speed Speed from 0 to 1
         * @return this for chaining
         */
        public PathBuilder setSpeed(double speed) {
            this.currentSpeed = Math.max(0, Math.min(1, speed));
            return this;
        }

        /**
         * Sets the tolerance for subsequent waypoints.
         *
         * @param positionMM Position tolerance in mm
         * @param headingRad Heading tolerance in radians
         * @return this for chaining
         */
        public PathBuilder setTolerance(double positionMM, double headingRad) {
            this.currentPositionTolerance = positionMM;
            this.currentHeadingTolerance = headingRad;
            return this;
        }

        /**
         * Adds a waypoint to the path.
         *
         * @param waypoint The waypoint to add
         * @return this for chaining
         */
        public PathBuilder addWaypoint(Waypoint waypoint) {
            waypoints.add(waypoint);
            currentPose = new Pose2d(waypoint.getPosition(), currentPose.getHeading());
            return this;
        }

        /**
         * Moves forward relative to current heading.
         *
         * @param distanceMM Distance in mm
         * @return this for chaining
         */
        public PathBuilder forward(double distanceMM) {
            double x = currentPose.getX() + distanceMM * Math.cos(currentPose.getHeading());
            double y = currentPose.getY() + distanceMM * Math.sin(currentPose.getHeading());
            return lineTo(x, y);
        }

        /**
         * Moves backward relative to current heading.
         *
         * @param distanceMM Distance in mm
         * @return this for chaining
         */
        public PathBuilder back(double distanceMM) {
            return forward(-distanceMM);
        }

        /**
         * Strafes right relative to current heading.
         *
         * @param distanceMM Distance in mm
         * @return this for chaining
         */
        public PathBuilder strafeRight(double distanceMM) {
            double perpAngle = currentPose.getHeading() - Math.PI / 2;
            double x = currentPose.getX() + distanceMM * Math.cos(perpAngle);
            double y = currentPose.getY() + distanceMM * Math.sin(perpAngle);
            return lineTo(x, y);
        }

        /**
         * Strafes left relative to current heading.
         *
         * @param distanceMM Distance in mm
         * @return this for chaining
         */
        public PathBuilder strafeLeft(double distanceMM) {
            return strafeRight(-distanceMM);
        }

        /**
         * Moves to an absolute position while maintaining heading.
         *
         * @param x Target X in mm
         * @param y Target Y in mm
         * @return this for chaining
         */
        public PathBuilder lineTo(double x, double y) {
            Waypoint wp = new Waypoint(x, y)
                    .setHeadingMode(Waypoint.HeadingMode.MAINTAIN)
                    .setSpeed(currentSpeed)
                    .setPositionTolerance(currentPositionTolerance)
                    .setHeadingTolerance(currentHeadingTolerance);
            return addWaypoint(wp);
        }

        /**
         * Moves to an absolute position with specific heading.
         *
         * @param x       Target X in mm
         * @param y       Target Y in mm
         * @param heading Target heading in radians
         * @return this for chaining
         */
        public PathBuilder lineToHeading(double x, double y, double heading) {
            Waypoint wp = new Waypoint(x, y, heading)
                    .setSpeed(currentSpeed)
                    .setPositionTolerance(currentPositionTolerance)
                    .setHeadingTolerance(currentHeadingTolerance);
            currentPose = new Pose2d(x, y, heading);
            return addWaypoint(wp);
        }

        /**
         * Moves to an absolute position while facing the direction of travel.
         *
         * @param x Target X in mm
         * @param y Target Y in mm
         * @return this for chaining
         */
        public PathBuilder splineTo(double x, double y) {
            Waypoint wp = new Waypoint(x, y)
                    .setHeadingMode(Waypoint.HeadingMode.PATH)
                    .setSpeed(currentSpeed)
                    .setPositionTolerance(currentPositionTolerance)
                    .setHeadingTolerance(currentHeadingTolerance);
            currentPose = new Pose2d(x, y, Math.atan2(y - currentPose.getY(), x - currentPose.getX()));
            return addWaypoint(wp);
        }

        /**
         * Moves to a position while facing a target point.
         *
         * @param x      Target X in mm
         * @param y      Target Y in mm
         * @param faceX  X coordinate to face
         * @param faceY  Y coordinate to face
         * @return this for chaining
         */
        public PathBuilder lineToFacing(double x, double y, double faceX, double faceY) {
            Waypoint wp = new Waypoint(x, y)
                    .setHeadingTarget(new Translation2d(faceX, faceY))
                    .setSpeed(currentSpeed)
                    .setPositionTolerance(currentPositionTolerance)
                    .setHeadingTolerance(currentHeadingTolerance);
            return addWaypoint(wp);
        }

        /**
         * Turns to face a specific heading without moving.
         *
         * @param heading Target heading in radians
         * @return this for chaining
         */
        public PathBuilder turnTo(double heading) {
            Waypoint wp = new Waypoint(currentPose.getX(), currentPose.getY(), heading)
                    .setSpeed(currentSpeed * 0.5) // Slower for pure rotation
                    .setPositionTolerance(currentPositionTolerance)
                    .setHeadingTolerance(currentHeadingTolerance);
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), heading);
            return addWaypoint(wp);
        }

        /**
         * Waits at the current position.
         *
         * @param seconds Duration to wait
         * @return this for chaining
         */
        public PathBuilder waitSeconds(double seconds) {
            Waypoint wp = new Waypoint(currentPose)
                    .setPositionTolerance(1000) // Large tolerance = immediate
                    .setActionDelay(seconds)
                    .setAction(() -> {}); // Empty action to trigger delay
            return addWaypoint(wp);
        }

        /**
         * Adds an action at the current position.
         *
         * @param action The action to run
         * @return this for chaining
         */
        public PathBuilder addAction(Runnable action) {
            if (!waypoints.isEmpty()) {
                waypoints.get(waypoints.size() - 1).setAction(action);
            }
            return this;
        }

        /**
         * Builds and sets the path on the follower.
         */
        public void build() {
            follower.setPath(waypoints);
        }

        /**
         * Gets the built waypoints without setting them.
         *
         * @return List of waypoints
         */
        public List<Waypoint> getWaypoints() {
            return new ArrayList<>(waypoints);
        }
    }
}
