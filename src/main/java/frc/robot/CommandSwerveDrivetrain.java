package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.util.ModifiedSignalLogger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Vision visionShooter = new Vision(VisionConstants.SHOOTER_CAMERA,VisionConstants.SHOOTER_CAMERA_TRANSFORM);
    private Vision visionIntake = new Vision(VisionConstants.SHOOTER_CAMERA,VisionConstants.SHOOTER_CAMERA_TRANSFORM);
    
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    private void visionEstimation(){
        // var visionEst1 = visionShooter.getEstimatedGlobalPose();
        // if (visionEst1.isEmpty()){
        //     return;
        // }
        // addVisionMeasurement(visionEst1.get().estimatedPose.toPose2d(), visionEst1.get().timestampSeconds, visionShooter.getEstimationStdDevs(visionEst1.get().estimatedPose.toPose2d()));

        // var visionEst2 = vision2.getEstimatedGlobalPose();
        // if (visionEst2.isEmpty()){
        //     return;
        // }
        // addVisionMeasurement(visionEst2.get().estimatedPose.toPose2d(), visionEst2.get().timestampSeconds, vision2.getEstimationStdDevs(visionEst2.get().estimatedPose.toPose2d()));
        var visionEstShooter = visionShooter.getEstimatedGlobalPose();
        if (visionEstShooter.isPresent()) {         
            addVisionMeasurement(visionEstShooter.get().estimatedPose.toPose2d(), visionEstShooter.get().timestampSeconds, visionShooter.getEstimationStdDevs(visionEstShooter.get().estimatedPose.toPose2d()));
        }

        var visionEstIntake = visionIntake.getEstimatedGlobalPose();
        if (visionEstIntake.isPresent()) {
            addVisionMeasurement(visionEstIntake.get().estimatedPose.toPose2d(), visionEstIntake.get().timestampSeconds, visionIntake.getEstimationStdDevs(visionEstIntake.get().estimatedPose.toPose2d()));
        }
    }

    @Override
    public void periodic(){
        visionEstimation();
        // System.out.println("Speaker Distance " + getDistanceFromSpeakerMeters());
        // System.out.println("Rotation " + getAngleFromSpeaker());
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0.4;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // getState of the robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0),
                                            new PIDConstants(5, 0, 0),
                    5.3,
            driveBaseRadius,
            new ReplanningConfig()),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                            
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            // Reference to this subsystem to set requirements // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }
    // @Override
    // public void +() {
    //     try {
    //         m_stateLock.writeLock().lock();

    //         m_fieldRelativeOffset = getState().Pose.getRotation();
    //     } finally {
    //         m_stateLock.writeLock().unlock();
    //     }
    // }

    // private void setYaw(Rotation2d yaw) {
    //     m_pigeon2.setYaw(yaw.getDegrees());
    // }
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    // private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    // private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //                 (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    //                 null,
    //                 this));

    // private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);
    
    // private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //                 (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
    //                 null,
    //                 this));

    // private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //                 (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    //                 null,
    //                 this));

    // public Command runDriveQuasiTest(Direction direction) {
    //     return m_driveSysIdRoutine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    //     return m_driveSysIdRoutine.dynamic(direction);
    // }

    // public Command runSteerQuasiTest(Direction direction) {
    //     return m_steerSysIdRoutine.quasistatic(direction);
    // }

    // public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    //     return m_steerSysIdRoutine.dynamic(direction);
    // }

    // public Command runDriveSlipTest() {
    //     return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble());
    }

    public Rotation2d getAngleFromSpeaker() {
        // var alliance = DriverStation.getAlliance();
        // if (!alliance.isPresent()) {
        //     return new Rotation2d(0);
        // }
        // if (alliance.get() == DriverStation.Alliance.Blue) {
        //     System.out.println(visionShooter.getTargetAngle(7));
        //     return Rotation2d.fromDegrees(visionShooter.getTargetAngle(7));
        // } else {
        //     return Rotation2d.fromDegrees(visionShooter.getTargetAngle(4));
        // }
        LimelightResults results =  LimelightHelpers.getLatestResults(LimelightConstants.LIMELIGHT_NAME);     

        if (results.targetingResults.valid) {
            return Rotation2d.fromDegrees(LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_NAME));
        }else{
            Pose2d speakerLocation;
            var alliance = DriverStation.getAlliance();
            if (!alliance.isPresent()) {
                return new Rotation2d(0);
            }
            if (alliance.get() == DriverStation.Alliance.Blue) {
                speakerLocation = new Pose2d(-0.0381, 5.547868, null);
            } else {
                speakerLocation = new Pose2d(16.579342, 5.547868, null);
            }
            
            double currentPoseX = getPose().getX();
            double currentPoseY = getPose().getY();
            double deltaX = currentPoseX - speakerLocation.getX();
            double deltaY = currentPoseY - speakerLocation.getY();

            double angleRadians = ((Math.atan2(deltaX, deltaY)*-1)-Math.PI/2)+Math.PI*1.5;

            // Convert the angle to Rotation2d
            Rotation2d rotation = Rotation2d.fromRadians(angleRadians);
            // System.out.println(rotation.getDegrees());
            return rotation;//Rotation2d.fromRadians(Math.atan((currentPoseX - speakerLocation.getX())/(currentPoseY - speakerLocation.getY())));
        }
    }

    public double getDistanceFromSpeakerMeters() {
        LimelightResults results =  LimelightHelpers.getLatestResults(LimelightConstants.LIMELIGHT_NAME);     

        if (results.targetingResults.valid) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.LIMELIGHT_NAME);
            NetworkTableEntry ty = table.getEntry("ty");
            double targetOffsetAngle_Vertical = ty.getDouble(0.0);

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = LimelightConstants.LIMELIGHT_CAMERA_TRANSFORM.getRotation().getY(); 

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = LimelightConstants.LIMELIGHT_CAMERA_TRANSFORM.getZ(); 

            // distance from the target to the floor
            double goalHeightInches = VisionConstants.K_TAG_LAYOUT.getTagPose(7).get().getZ(); 

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            return distanceFromLimelightToGoalInches;
        }else{
            return -1;
        }
    }


    public void seedFieldRelative(double degrees) {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = new Rotation2d(Units.degreesToRadians(getState().Pose.getRotation().getDegrees()+degrees));
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }
    // public Rotation2d getYawNoOffset(){
    //     return m_pigeon2.getRotation2d();
    // }
    
}
