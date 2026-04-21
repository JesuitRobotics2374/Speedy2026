package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Critical Generic Constants
    public static final double MAX_SPEED = 0.6; // kSpeedAt12Volts desired top speed
    public static final double MAX_SPEED_TURBO = 0.65;
    public static final double MAX_ANGULAR_RATE = 0.45; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // General Constants
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // PhotonVision
    public static final int numberOfCams = 0;

    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;

    public static final int PIGEON_ID = 0;

    
}
