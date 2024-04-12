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
import java.util.function.Supplier;

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
import frc.robot.constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.util.PathOnTheFly;
import frc.robot.util.PathOnTheFly.AutoToPoint;
import frc.robot.util.PathOnTheFly.PathConfig;
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
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * 0.90 * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * 0.90 * MaxSpeed)
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
    // operatorController.y().onTrue(groupCommands.wheelRadiusCommand());
    operatorController.x().whileTrue(intake.setVelocity(15));
    // operatorController.y().whileTrue(getAutoToPoint().andThen(groupCommands.loadAndShoot()));
    operatorController.leftBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(60)));
    operatorController.rightBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(0)));
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    // driverController.getHID().se\tRumble(RumbleType.kBothRumble, 1);
    Supplier<Pose2d> teamID = ()->{
      if (DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
          return LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
        }
      }
      return LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
    };
    new Trigger(()-> drivetrain.getDistanceFromPoseMeters(teamID.get()) > 1.95072 && drivetrain.getDistanceFromPoseMeters(teamID.get()) < 2.7432)
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

    PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),3,0);
    PathOnTheFly.addConfig(pathConfig,0);
    autoChooser.addOption("Conditional Auto", getAutonomousCommandSpecial());
    autoChooser.addOption("OnTheFlyAuto", getOnTheFlyAuto());
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
    // return getAutonomousCommandSpecial();
  }
  public Command getAutonomousCommandSpecial() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180)));
    }else{
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    }
    
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("S-A 4.5 piece continuous")).andThen(Commands.either(
      AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("6 shoot"),
            new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)),
            3.0 
    ),AutoBuilder.followPath(PathPlannerPath.fromPathFile("6 to 7")).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("7 shoot"))),()->intake.isPiecePresent()));
    // PathPlannerPath.fromPathFile("S-A 4.5 piece continuous");
  }
  public Command getOnTheFlyAuto() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180)));
    }else{
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    }
    return AutoToPoint.getToPoint(7.11,7.44,0,PathOnTheFly.getConfig(0))
    .andThen(Commands.either(Commands.deadline(groupCommands.intake(),groupCommands.alignToPiece())
    ,AutoToPoint.getToPoint(7.72,6.24,-37.12,PathOnTheFly.getConfig(0)).andThen(Commands.deadline(groupCommands.intake(),groupCommands.alignToPiece()))
    ,()->drivetrain.isPiecePresent() && CommandSwerveDrivetrain.poseWithinRange(drivetrain.getPiecePose().get(),new Pose2d(8.29,7.42,new Rotation2d()),.2)))
    .andThen(AutoToPoint.getToPoint(5.19,6.18,-172.21,PathOnTheFly.getConfig(0)))
    .andThen(Commands.deadline(Commands.waitSeconds(3),groupCommands.getInRange())).andThen(groupCommands.loadAndShoot());
  }
}