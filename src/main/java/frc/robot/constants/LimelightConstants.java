package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MultiLinearInterpolator;

public class LimelightConstants {
    /* 5027 https://github.com/FRC5727/SwervyBoi/blob/76bf195e5332ee201a1d0d766fbc0b57b428d485/src/main/java/frc/robot/Constants.java */
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double MAX_XY_ERROR = 1.0;
    public static final Transform3d LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0,-.297, 0.25146), new Rotation3d(0, Units.degreesToRadians(33.5), Units.degreesToRadians(0)));

    public static final String AMP_CAM = "ampcam";
    public static final double MAX_XY_ERROR_AMP_CAM = 1.0;
    public static final Transform3d AMP_CAM_TRANSFORM =
        new Transform3d(new Translation3d(0,0, 0), new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));

    public static final AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees//actually maybe radians) std deviation}
      {0, 0.001, 0.001, 1},
      {1.5, 0.01, 0.01, 3},
      {3, 0.7, 0.7, 10},
      {4.5, 3, 3, 100},
      {6, 8, 8, 1000}
    };
    public static final MultiLinearInterpolator ONE_APRIL_TAG_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(ONE_APRIL_TAG_LOOKUP_TABLE);

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.001, 0.001, .25},
      {1.5, 0.01, 0.01, .6},
      {3, 0.4, 0.4, 10},
      {4.5, 2, 2, 100},
      {6, 6, 6, 1000}
    };
    public static final MultiLinearInterpolator TWO_APRIL_TAG_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(TWO_APRIL_TAG_LOOKUP_TABLE);

}
