// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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


  /* Subsystems */
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Candle candle = new Candle();
  private final FactoryCommands groupCommands = new FactoryCommands(arm, shooter, intake, candle, drivetrain);
  

  private void configureBindings() {
    /* Setup Default Commands */

    shooter.enableDefault();
    intake.setDefaultCommand(intake.stop());
    candle.setDefaultCommand(candle.idleLED());
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
    ));

    /* Controller Bindings */

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.rightBumper().whileTrue(groupCommands.intake()).whileFalse(Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState())));
    driverController.leftBumper().onTrue(groupCommands.shoot());
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));


    driverController.x().whileTrue(groupCommands.alignToSpeaker(() -> -driverController.getLeftY() * MaxSpeed, () -> -driverController.getLeftX() * MaxSpeed));
    driverController.a().whileTrue(groupCommands.alignToAmp(() -> -driverController.getLeftY() * MaxSpeed, () -> -driverController.getLeftX() * MaxSpeed)).and(() -> !driverController.x().getAsBoolean());
    // driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    
    operatorController.a().onTrue(groupCommands.switchModes());
    operatorController.x().whileTrue(intake.setVelocity(15));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // new Trigger()
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
    NamedCommands.registerCommand("loadAndShoot", groupCommands.loadAndShoot());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}