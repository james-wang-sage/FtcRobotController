package org.firstinspires.ftc.teamcode.pickle.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pickle.field.Alliance;
import org.firstinspires.ftc.teamcode.pickle.field.DecodeField;
import org.firstinspires.ftc.teamcode.pickle.field.FieldConstants;
import org.firstinspires.ftc.teamcode.pickle.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pickle.geometry.Translation2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * AprilTag-based robot localization for DECODE field.
 *
 * <h2>How AprilTag Localization Works</h2>
 * <p>AprilTags are fiducial markers with known positions on the field. When the camera
 * detects a tag, it can calculate the camera's position relative to the tag. Combined
 * with the tag's known field position, we can determine the robot's field position.</p>
 *
 * <h2>DECODE Field Tags</h2>
 * <pre>
 * Tag ID | Location        | Use for Localization?
 * -------+-----------------+----------------------
 *   24   | Red Goal        | Yes - primary target
 *   20   | Blue Goal       | Yes - primary target
 *   21   | Obelisk Face 1  | No - moving object
 *   22   | Obelisk Face 2  | No - moving object
 *   23   | Obelisk Face 3  | No - moving object
 * </pre>
 *
 * <h2>Camera Mounting</h2>
 * <p>The camera position on the robot must be known for accurate localization.
 * The position is relative to the robot center:</p>
 * <ul>
 *   <li>+X = forward from robot center</li>
 *   <li>+Y = left from robot center</li>
 *   <li>+Z = up from robot center</li>
 * </ul>
 *
 * <h2>Accuracy Considerations</h2>
 * <ul>
 *   <li>Closer tags give more accurate readings</li>
 *   <li>Tags viewed at an angle are less accurate</li>
 *   <li>Goal tags are preferred over obelisk tags (which move)</li>
 *   <li>Use sensor fusion to blend AprilTag data with odometry</li>
 * </ul>
 *
 * <h2>Usage</h2>
 * <pre>
 * // In init()
 * localizer = new AprilTagLocalizer(hardwareMap, "Webcam 1");
 * localizer.setCameraPosition(0, 0, 457.2); // Camera centered, 18 inches (457.2mm) up
 *
 * // In loop()
 * Pose2d visionPose = localizer.getEstimatedPose(alliance);
 * if (visionPose != null) {
 *     odometry.correctPose(visionPose, 0.3); // Blend with odometry
 * }
 * </pre>
 */
public class AprilTagLocalizer {

    private final AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Camera mounting position relative to robot center (in mm)
    // Camera is mounted front-center at the highest frame (18 inches up)
    private double cameraXOffset = 0;    // Forward from center (front-mounted, centered on frame)
    private double cameraYOffset = 0;    // Left from center (centered)
    private double cameraZOffset = 457.2;  // Height above ground (18 inches = 457.2mm)
    private double cameraPitch = 0;      // Tilt angle in radians (negative = looking down)
    private double cameraYaw = 0;        // Rotation in radians (positive = turned left)

    // Detection filtering
    private double maxDetectionRange = 2000; // mm - ignore detections beyond this
    private double minDecisionMargin = 10;   // Confidence threshold

    // Last valid pose for debugging
    private Pose2d lastValidPose = null;
    private long lastDetectionTime = 0;

    /**
     * Creates a new AprilTagLocalizer.
     *
     * @param hardwareMap The hardware map
     * @param cameraName  The name of the webcam in the hardware config
     */
    public AprilTagLocalizer(HardwareMap hardwareMap, String cameraName) {
        // Create AprilTag processor with DECODE tags
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // Create vision portal
        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                    .addProcessor(aprilTagProcessor)
                    .enableLiveView(true)
                    .build();
        } catch (Exception e) {
            visionPortal = null;
        }
    }

    /**
     * Sets the camera position on the robot.
     *
     * @param xOffset Forward offset from robot center (mm)
     * @param yOffset Left offset from robot center (mm)
     * @param zOffset Height above ground (mm)
     */
    public void setCameraPosition(double xOffset, double yOffset, double zOffset) {
        this.cameraXOffset = xOffset;
        this.cameraYOffset = yOffset;
        this.cameraZOffset = zOffset;
    }

    /**
     * Sets the camera orientation.
     *
     * @param pitchRadians Tilt angle (negative = looking down)
     * @param yawRadians   Rotation angle (positive = turned left)
     */
    public void setCameraOrientation(double pitchRadians, double yawRadians) {
        this.cameraPitch = pitchRadians;
        this.cameraYaw = yawRadians;
    }

    /**
     * Sets the maximum range for valid detections.
     *
     * @param rangeMMM Maximum detection range in mm
     */
    public void setMaxDetectionRange(double rangeMMM) {
        this.maxDetectionRange = rangeMMM;
    }

    /**
     * Gets all current AprilTag detections.
     *
     * @return List of detections, or empty list if none
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) {
            return new ArrayList<>();
        }
        return aprilTagProcessor.getDetections();
    }

    /**
     * Gets detections filtered to only goal tags (ignoring obelisk tags).
     *
     * @return List of goal tag detections
     */
    public List<AprilTagDetection> getGoalDetections() {
        List<AprilTagDetection> goalDetections = new ArrayList<>();
        for (AprilTagDetection detection : getDetections()) {
            if (DecodeField.isGoalTag(detection.id)) {
                goalDetections.add(detection);
            }
        }
        return goalDetections;
    }

    /**
     * Gets the best detection for localization (closest goal tag with good confidence).
     *
     * @param alliance Preferred alliance (prioritizes that goal)
     * @return Best detection, or null if none available
     */
    public AprilTagDetection getBestDetection(Alliance alliance) {
        int preferredTagId = DecodeField.getGoalAprilTagId(alliance);

        AprilTagDetection best = null;
        double bestScore = Double.MAX_VALUE;

        for (AprilTagDetection detection : getGoalDetections()) {
            // Skip if no pose data available
            if (detection.ftcPose == null) continue;

            // Calculate detection range
            double range = Math.sqrt(
                    detection.ftcPose.x * detection.ftcPose.x +
                    detection.ftcPose.y * detection.ftcPose.y +
                    detection.ftcPose.z * detection.ftcPose.z
            );

            // Skip if too far
            if (range > maxDetectionRange) continue;

            // Skip if low confidence
            if (detection.decisionMargin < minDecisionMargin) continue;

            // Score: lower is better. Prefer closer tags and alliance's goal
            double score = range;
            if (detection.id == preferredTagId) {
                score *= 0.5; // 50% bonus for our goal
            }

            if (score < bestScore) {
                bestScore = score;
                best = detection;
            }
        }

        return best;
    }

    /**
     * Estimates the robot's field pose from AprilTag detections.
     *
     * <p>This uses the robot's pose data from the SDK, which accounts for the
     * camera's position on the robot. The pose is in field coordinates.</p>
     *
     * @param alliance The alliance (for tag prioritization)
     * @return Estimated pose, or null if no valid detection
     */
    public Pose2d getEstimatedPose(Alliance alliance) {
        AprilTagDetection detection = getBestDetection(alliance);

        if (detection == null || detection.robotPose == null) {
            return null;
        }

        // The SDK provides robotPose in field coordinates
        Position pos = detection.robotPose.getPosition();
        YawPitchRollAngles orient = detection.robotPose.getOrientation();

        // Convert to our Pose2d format (mm and radians)
        // Note: SDK already handles camera offset transformation
        double x = pos.x;  // Already in mm based on processor config
        double y = pos.y;
        double heading = orient.getYaw(AngleUnit.RADIANS);

        lastValidPose = new Pose2d(x, y, heading);
        lastDetectionTime = System.currentTimeMillis();

        return lastValidPose;
    }

    /**
     * Gets the field position of a detected goal tag.
     *
     * @param tagId The AprilTag ID
     * @return The tag's field position, or null if unknown
     */
    public Translation2d getTagFieldPosition(int tagId) {
        if (tagId == DecodeField.APRILTAG_RED_GOAL) {
            return DecodeField.getGoalCenter(Alliance.RED);
        } else if (tagId == DecodeField.APRILTAG_BLUE_GOAL) {
            return DecodeField.getGoalCenter(Alliance.BLUE);
        }
        return null;
    }

    /**
     * Calculates robot pose from a detection using manual transformation.
     *
     * <p>Use this if robotPose is not available in the detection.</p>
     *
     * @param detection The AprilTag detection
     * @return Calculated robot pose, or null if calculation fails
     */
    public Pose2d calculatePoseFromDetection(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;

        // Get the tag's known field position
        Translation2d tagFieldPos = getTagFieldPosition(detection.id);
        if (tagFieldPos == null) return null;

        // ftcPose gives camera pose relative to tag
        // x = right, y = forward, z = up (in tag's frame)
        double camToTagX = detection.ftcPose.x;  // Right/left
        double camToTagY = detection.ftcPose.y;  // Forward/back
        double camToTagYaw = detection.ftcPose.yaw;  // Tag's apparent rotation

        // Camera position relative to tag in field frame
        // Tag faces into the field, so we need to rotate
        double tagYaw = getTagFieldYaw(detection.id);

        // Transform camera-to-tag vector to field frame
        double cos = Math.cos(tagYaw);
        double sin = Math.sin(tagYaw);

        double camFieldX = tagFieldPos.getX() - (camToTagY * cos - camToTagX * sin);
        double camFieldY = tagFieldPos.getY() - (camToTagY * sin + camToTagX * cos);

        // Robot heading relative to field
        double robotHeading = tagYaw - camToTagYaw + Math.PI;

        // Account for camera offset on robot
        double robotFieldX = camFieldX - cameraXOffset * Math.cos(robotHeading) + cameraYOffset * Math.sin(robotHeading);
        double robotFieldY = camFieldY - cameraXOffset * Math.sin(robotHeading) - cameraYOffset * Math.cos(robotHeading);

        return new Pose2d(robotFieldX, robotFieldY, Pose2d.normalizeAngle(robotHeading));
    }

    /**
     * Gets the field-relative yaw of a tag (which way it's facing).
     *
     * @param tagId The tag ID
     * @return Yaw in radians
     */
    private double getTagFieldYaw(int tagId) {
        // Red goal faces toward -X (into the field from red wall)
        if (tagId == DecodeField.APRILTAG_RED_GOAL) {
            return Math.PI; // Facing -X
        }
        // Blue goal faces toward +X (into the field from blue wall)
        if (tagId == DecodeField.APRILTAG_BLUE_GOAL) {
            return 0; // Facing +X
        }
        return 0;
    }

    /**
     * Gets the time since the last valid detection.
     *
     * @return Milliseconds since last detection, or Long.MAX_VALUE if never detected
     */
    public long getTimeSinceLastDetection() {
        if (lastDetectionTime == 0) return Long.MAX_VALUE;
        return System.currentTimeMillis() - lastDetectionTime;
    }

    /**
     * Gets the last valid pose (even if no current detection).
     *
     * @return Last valid pose, or null if never detected
     */
    public Pose2d getLastValidPose() {
        return lastValidPose;
    }

    /**
     * Checks if the localizer has a recent valid detection.
     *
     * @param maxAge Maximum age in milliseconds
     * @return true if a valid detection exists within maxAge
     */
    public boolean hasRecentDetection(long maxAge) {
        return getTimeSinceLastDetection() <= maxAge;
    }

    /**
     * Enables or disables the vision processing.
     *
     * @param enabled true to enable, false to pause
     */
    public void setEnabled(boolean enabled) {
        if (visionPortal != null) {
            if (enabled) {
                visionPortal.resumeStreaming();
            } else {
                visionPortal.stopStreaming();
            }
        }
    }

    /**
     * Closes the vision portal. Call this when done with localization.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Gets diagnostic information.
     *
     * @return Formatted diagnostic string
     */
    public String getDiagnostics() {
        List<AprilTagDetection> detections = getDetections();
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Tags: %d | ", detections.size()));

        if (lastValidPose != null) {
            sb.append(String.format("Last: (%.0f, %.0f, %.1f°) %dms ago",
                    lastValidPose.getX(),
                    lastValidPose.getY(),
                    Math.toDegrees(lastValidPose.getHeading()),
                    getTimeSinceLastDetection()));
        } else {
            sb.append("No valid pose");
        }

        return sb.toString();
    }
}
