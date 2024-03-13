package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.sql.Driver;
import java.util.function.Supplier;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    private Vision vision = new Vision(VisionConstants.SHOOTER_CAMERA,VisionConstants.SHOOTER_CAMERA_TRANSFORM);
    
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
        var visionEst1 = vision.getEstimatedGlobalPose();
        if (visionEst1.isEmpty()){
            return;
        }
        addVisionMeasurement(visionEst1.get().estimatedPose.toPose2d(), visionEst1.get().timestampSeconds, vision.getEstimationStdDevs(visionEst1.get().estimatedPose.toPose2d()));


        // var visionEst2 = vision2.getEstimatedGlobalPose();
        // if (visionEst2.isEmpty()){
        //     return;
        // }
        // addVisionMeasurement(visionEst2.get().estimatedPose.toPose2d(), visionEst2.get().timestampSeconds, vision2.getEstimationStdDevs(visionEst2.get().estimatedPose.toPose2d()));
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
    // public void seedFieldRelative(Pose2d location) {
    //     try {
    //         m_stateLock.writeLock().lock();

    //         m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
    //         seedFieldRelative();
    //         // setYaw(location.getRotation());
    //         /* We need to update our cached pose immediately so that race conditions don't happen */
    //         m_cachedState.Pose = location;
    //     } finally {
    //         m_stateLock.writeLock().unlock();
    //     }
    // }

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
        var alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return new Rotation2d(0);
        }
        if (alliance.get() == DriverStation.Alliance.Blue) {
            System.out.println(vision.getTargetAngle(7));
            return Rotation2d.fromDegrees(vision.getTargetAngle(7));
        } else {
            return Rotation2d.fromDegrees(vision.getTargetAngle(4));
        }
    }

    public double getDistanceFromSpeakerMeters() {
        var alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return 0;
        }
        if (alliance.get() == DriverStation.Alliance.Blue) {
            System.out.println(vision.getTargetAngle(7));
            return vision.getTargetDist(7);
        } else {
            System.out.println(vision.getTargetAngle(4));
            return vision.getTargetDist(4);
        }
    }

}
