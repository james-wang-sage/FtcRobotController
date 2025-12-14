package org.firstinspires.ftc.teamcode.pickle;

/**
 * Central place for Driver Station Robot Configuration device names used by Pickle OpModes.
 *
 * Why:
 * - Avoid typos (stringly-typed hardware names)
 * - Make it easy to change a config name once and keep Auto/TeleOp consistent
 */
public final class PickleHardwareNames {
    private PickleHardwareNames() {}

    // Drive motors (mecanum)
    public static final String FRONT_LEFT_MOTOR = "front_left";
    public static final String FRONT_RIGHT_MOTOR = "front_right";
    public static final String BACK_LEFT_MOTOR = "back_left";
    public static final String BACK_RIGHT_MOTOR = "back_right";

    // Launcher + feeders
    public static final String LAUNCHER_MOTOR = "launcher";
    public static final String LEFT_FEEDER_SERVO = "left_feeder";
    public static final String RIGHT_FEEDER_SERVO = "right_feeder";

    // Vision
    public static final String WEBCAM_NAME = "Webcam 1";
}

