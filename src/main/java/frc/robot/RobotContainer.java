 // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = vision1.update();
        // if (optionalEstimatedPoseRight.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = vision2.update();
        // if (optionalEstimatedPoseLeft.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // poseEstimator.update(/*ccw gyro rotation*/, /*module positions array*/);// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FactoryCommands;
import frc.robot.constants.GeneralConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.TRANSLATIONAL_DEADBAND).withRotationalDeadband(TunerConstants.ROTATIONAL_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);


  public static Rotation2d autoAngleOffset;
  /* Subsystems */
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  // private final Candle candle = new Candle();
  private final FactoryCommands groupCommands = new FactoryCommands(arm, shooter, intake, drivetrain, driverController);
  

  private void configureBindings() {
    /* Setup Default Commands */

    shooter.enableDefault();
    intake.setDefaultCommand(intake.stop());
    // candle.setDefaultCommand(candle.idleLED());
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * 0.85 * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * 0.85 * MaxSpeed)
            .withRotationalRate(-driverController.getRightX() *2/3 * MaxAngularRate)
    ));

    /* Controller Bindings */

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.rightBumper().whileTrue(groupCommands.intake()).whileFalse(Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState())));
    driverController.leftBumper().onTrue(groupCommands.shoot());
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.leftTrigger(.5).whileTrue(groupCommands.getInRange())
    .onTrue(Commands.runOnce(()->shooter.setIdleSpeed(60)))
    .onFalse(Commands.runOnce(()->shooter.setIdleSpeed(0)));

    driverController.rightStick().whileTrue(groupCommands.alignToCorner());
    driverController.a().whileTrue(groupCommands.alignToAmp());//.and(() -> driverController.x().getAsBoolean());
    driverController.b().whileTrue(groupCommands.alignToPiece());
    // driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    
    operatorController.a().onTrue(groupCommands.switchModes());
    operatorController.x().whileTrue(intake.setVelocity(15));
    // operatorController.y().whileTrue(getAutoToPoint().andThen(groupCommands.loadAndShoot()));
    operatorController.leftBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(60)));
    operatorController.rightBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(0)));
    
    // operatorController.leftBumper().onTrue(groupCommands.loadAndShootOperator());
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    // driverController.getHID().se\tRumble(RumbleType.kBothRumble, 1);
    DoubleSupplier teamID = ()->{
      if (DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
          return 4;
        }
      }
      return 7;
    };
    new Trigger(()-> drivetrain.getDistanceFromTagMeters(teamID.getAsDouble()) > 1.95072 && drivetrain.getDistanceFromTagMeters(teamID.getAsDouble()) < 2.7432)
      .whileTrue(Commands.runOnce(()->driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .whileFalse(Commands.runOnce(()->driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
    new Trigger(()->DriverStation.isTeleop()).and(()->{
      var alliance = DriverStation.getAlliance();
      if (!alliance.isPresent()){
        return true;
      }
      return DriverStation.getAlliance().get().equals(Alliance.Red);
    }).onTrue(Commands.runOnce(()->drivetrain.seedFieldRelative(180-drivetrain.getPose().getRotation().getDegrees())))
    .onFalse(Commands.runOnce(()->drivetrain.seedFieldRelative(360-drivetrain.getPose().getRotation().getDegrees())));
  }

  public RobotContainer() {
    drivetrain.configurePathPlanner();
    configureBindings();
    configureAutonomousCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", groupCommands.intakeMainAuto());
    NamedCommands.registerCommand("setIdleSpeed", Commands.runOnce(()->shooter.setAutoIdleSpeed(40)));
    NamedCommands.registerCommand("loadAndShoot", groupCommands.loadAndShootAuto());
    NamedCommands.registerCommand("loadAndShootLinear", groupCommands.loadAndShoot());
    NamedCommands.registerCommand("loadAndShootThree", groupCommands.loadAndShootAutoSecondary());
    NamedCommands.registerCommand("loadAndShootFour", groupCommands.loadAndShootAutoTertiary());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }



  private Command getAutoToPoint(){
    Pose2d targetPose = new Pose2d(9.2, 1, Rotation2d.fromDegrees(325));


    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));


    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;
  }
  private Command getAutoToPath(){
    PathPlannerPath path = PathPlannerPath.fromPathFile("Rush Hour 1");


    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));


    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints,
            3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;
  }
}