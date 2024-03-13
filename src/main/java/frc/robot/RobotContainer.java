// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FactoryCommands;
import frc.robot.constants.GeneralConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = GeneralConstants.MAX_ANGULAR_RATE; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Candle candle = new Candle();
  private FactoryCommands factoryCommands = new FactoryCommands(arm, shooter, intake, candle, drivetrain);





  private void configureBindings() {
    shooter.enableDefault();
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            // .withRotationalRate(-driverController.getRawAxis(2)*MaxAngularRate)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
    intake.setDefaultCommand(intake.stop());

    Command candleDist = Commands.run(()->factoryCommands.getDistCandle());
    // reset the field-centric heading on left bumper press
    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.rightBumper().whileTrue(factoryCommands.intake()).whileFalse(Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState())));
    driverController.leftBumper().onTrue(factoryCommands.shoot());
    driverController.a().whileTrue(factoryCommands.alignToAmp(() -> -driverController.getLeftY() * MaxSpeed, () -> -driverController.getLeftX() * MaxSpeed));
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.x().whileTrue(factoryCommands.alignToSpeaker(() -> -driverController.getLeftY() * MaxSpeed, () -> -driverController.getLeftX() * MaxSpeed));
    
    operatorController.b().whileTrue(getAutoToPath());
    operatorController.y().whileTrue(getAutoToPoint());
    operatorController.a().onTrue(factoryCommands.switchModes());
    operatorController.x().whileTrue(intake.setVelocity(15));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  public RobotContainer() {
    drivetrain.configurePathPlanner();
    configureBindings();
    configureAutonomousCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }
  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", factoryCommands.intakeMainAuto());
    NamedCommands.registerCommand("loadAndShoot", factoryCommands.loadAndShoot());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command getAutoToPoint(){
    Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));


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