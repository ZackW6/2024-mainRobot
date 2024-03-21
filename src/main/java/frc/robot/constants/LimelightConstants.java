package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    /* 5027 https://github.com/FRC5727/SwervyBoi/blob/76bf195e5332ee201a1d0d766fbc0b57b428d485/src/main/java/frc/robot/Constants.java */
    public static final String LIMELIGHT_NAME = "";
    public static final double MAX_XY_ERROR = 1.0;
    public static final Transform3d LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0,-.297, 0.25146), new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(0)));

    public static final String LIMELIGHT3G_NAME = "newLimelightCameraNameHere";
    public static final double MAX_XY_ERROR3G = 1.0;
    public static final Transform3d LIMELIGHT3G_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0,0, 0), new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));

    public static final AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.145, 1.20, 30},
      {4.5, 0.75, 5.0, 90},
      {6, 1.0, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 5},
      {1.5, 0.02, 0.02, 5},
      {3, 0.04, 0.04, 15},
      {4.5, 0.1, 0.1, 30},
      {6, 0.3, 0.3, 60}
    };
}
