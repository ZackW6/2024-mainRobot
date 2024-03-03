package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.io.IOException;



public final class Constants {
    public static final double stickDeadband = 0;
    public static String mainCanbus = "rio";

    public static final class AutoConstants { 
        public static final HolonomicPathFollowerConfig HolonomicConfigs = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(11, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.323, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 4;
        public static final double kPYController = 4;
        public static final double kPThetaController = 4.5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class ArmConstants{
        public static final int armMotorID = 10;
        public static final int CANCoderID = 10;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 300;
        public static final double kI = 0.1;
        public static final double kD = 55;
        public static final double kG = 8.574609375;//8.4749;

        // TODO: Make the arm positive when it goes up.
        public static final double cruiseVelocity = 400;
        public static final double maxAcceleration = 1000;
		public static final double jerk = 9999;

        
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(.385);//.314);//
        

        /* Arm Current Limiting */ //TODO: Change these values
        public static final int armCurrentLimit = 65;
        public static final int armSupplyCurrentThreshold = 10;
        public static final int armCurrentThreshold = 70;
        public static final double armCurrentThresholdTime = 0.1;
        public static final boolean armEnableCurrentLimit = true;
        public static final boolean armStatorCurrentLimitEnable = true;
		public static final double armStatorCurrentLimit = 200;
        
    }

    public static class IntakeConstants{    
        public static final int intakeMotorID = 11;
        public static final int limitSwitchID1 = 0;
        public static final int  limitSwicthID2 = 1;

        //TODO: Run SysID for shooter
        //TODO: Tune Shooter PID
        // Add 0.25 V output to overcome static friction

        public static final double kS = 0; // An error of 1 rps results in 0.11 V output
        public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
        public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double kP = 10; // This will need to be tuned after feedforward
        public static final double kI = 0; // For flywheels, this should be 0
        public static final double kD = 0; // For flywheels, this should be 0

        /* Intake Current Limiting */
        public static final int intakeCurrentLimit = 25;
        public static final int intakeSupplyCurrentThreshold = 10;
        public static final int intakeCurrentThreshold = 40;
        public static final double intakeCurrentThresholdTime = 0.1;
        public static final boolean intakeEnableCurrentLimit = true;
    }

    public static class ShooterConstants{
        public static final int leftShooterMotorID = 20;
        public static final int rightShooterMotorID = 21;
        
        //TODO: Run SysID for shooter
        //TODO: Tune Shooter PID https://docs.wpilib.org/en/latest/docs/software/advanced-controls/introduction/tuning-flywheel.html
     
        public static final double kS = 0.240; // Add 0.25 V output to overcome static friction
        public static final double kV = .11175; // A velocity target of 1 rps results in 0.12 V output
        public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double kP = 0.048; // This will need to be tuned after feedforward
        public static final double kI = 0; // For flywheels, this should be 0
        public static final double kD = 0; // For flywheels, this should be 0

        /* Shooter Current Limiting */
        public static final int shooterCurrentLimit = 25;
        public static final int shooterSupplyCurrentThreshold = 10;
        public static final int shooterCurrentThreshold = 40;
        public static final double shooterCurrentThresholdTime = 0.1;
        public static final boolean shooterEnableCurrentLimit = true;
    }
    public class VisionConstants {
        public static final String ShooterCamera = "ShooterCam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d ShooterCamTransform =
            new Transform3d(new Translation3d(-.231775,-.290306, .431), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180)));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 10);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    
    
    
        public static final String IntakeCamera = "IntakeCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d IntakeCamTransform =
                new Transform3d(new Translation3d(0.231775, -.139434, 0.538), new Rotation3d(0, 0, Math.toRadians(0)));
        // The standard deviations of our vision estimated poses, which affect correction rate
    }

    public static final class LimelightConstants {
        /* 5027 https://github.com/FRC5727/SwervyBoi/blob/76bf195e5332ee201a1d0d766fbc0b57b428d485/src/main/java/frc/robot/Constants.java */
        public static final String limelightName = "limelight";
        public static final double maxXYError = 1.0;
    
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

    
}
