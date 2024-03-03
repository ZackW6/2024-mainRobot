package frc.robot;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    Vision vision1 = new Vision(VisionConstants.ShooterCamera,VisionConstants.ShooterCamTransform);
    Vision vision2 = new Vision(VisionConstants.IntakeCamera, VisionConstants.IntakeCamTransform);
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
        var visionEst1 = vision1.getEstimatedGlobalPose();
        if (visionEst1.isEmpty()){
            return;
        }
        addVisionMeasurement(visionEst1.get().estimatedPose.toPose2d(), visionEst1.get().timestampSeconds, vision1.getEstimationStdDevs(visionEst1.get().estimatedPose.toPose2d()));

        var visionEst2 = vision2.getEstimatedGlobalPose();
        if (visionEst2.isEmpty()){
            return;
        }
        addVisionMeasurement(visionEst2.get().estimatedPose.toPose2d(), visionEst2.get().timestampSeconds, vision2.getEstimationStdDevs(visionEst2.get().estimatedPose.toPose2d()));
    }

    @Override
    public void periodic(){
        // vision.logTelemetry();
        visionEstimation();
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // getState of the robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
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
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }


    // public Command aimAt(XboxController xboxController){
    //         // Vision-alignment mode
    //         // Query the latest result from PhotonVision
    //         var result = vision2.getLatestResult();
    //         double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    //         double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    //         if (result.hasTargets()) {
    //             Units.degreesToRadians(result.getBestTarget().getPitch());

    //             // Use this range as the measurement we give to the PID controller.
    //             // -1.0 required to ensure positive PID controller effort _increases_ range
    //             // Drivetrain will execute this command periodically
    //             rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //             Commands.run(
    //                 applyRequest(() -> withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                                     // negative Y (forward)
    //                         .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                         .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //             ));
    //             // Also calculate angular power
    //             // -1.0 required to ensure positive PID controller effort _increases_ yaw
    //         } else {
    //             // If we have no targets, stay still.
    //             setDefaultCommand( // Drivetrain will execute this command periodically
    //                 applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                                 // negative Y (forward)
    //                     .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //                 ));
    //         }
    //         forwardSpeed = -xboxController.getRightY();
    //         rotationSpeed = xboxController.getLeftX();
    //     }

}
