// package frc.robot.constants;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.util.Units;

// public class VisionConstants {
//     public static final String SHOOTER_CAMERA = "ShooterCam";
//     // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
//     public static final Transform3d SHOOTER_CAMERA_TRANSFORM =
//         new Transform3d(new Translation3d(.5-(0.3556*3.4),-.1, .466), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180)));
//     // The layout of the AprilTags on the field
//     // new Transform3d(new Translation3d(-.231775,-.290306, .466), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180)));
//     public static final AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//     // The standard deviations of our vision estimated poses, which affect correction rate
//     // (Fake values. Experiment and determine estimation noise on an actual robot.)
//     public static final Matrix<N3, N1> K_SINGLE_TAG_STD_DEVS = VecBuilder.fill(6, 6, 10);
//     public static final Matrix<N3, N1> K_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);




//     public static final String INTAKE_CAMERA = "IntakeCamera";
//     // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
//     public static final Transform3d INTAKE_CAMERA_TRANSFORM =
//             new Transform3d(new Translation3d(0.231775, -.139434, 0.538), new Rotation3d(0, 0, Math.toRadians(0)));
//     // The standard deviations of our vision estimated poses, which affect correction rate
// }
