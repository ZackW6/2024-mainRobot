// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPositions;
import frc.robot.subsystems.Arm.ArmState;

public class GroupCommands  {
  private Arm arm;
  private Shooter shooter;
  private Intake intake;
  private Candle candle;
  private boolean bool = true;
  private CommandSwerveDrivetrain drivetrain;
  public GroupCommands(Arm arm, Shooter shooter, Intake intake, Candle candle, CommandSwerveDrivetrain drivetrain){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    this.candle = candle;
    this.drivetrain = drivetrain;
  }
  public Command intake(){
    return Commands.deadline(intake.intakePiece(),arm.setArmDegree(ArmPositions.Intake))
    .andThen(candle.pickUpLights())
    .andThen(Commands.deadline(Commands.waitSeconds(1),intake.setVelocity(5),arm.setArmDegree(ArmPositions.Load)))
    .andThen(intake.stop())
    .andThen(candle.idleLED());
  }
  public Command ampShot(){
    // return Commands.deadline(Commands.waitSeconds(2)
    // ,Commands.waitSeconds(1).alongWith(intake.setVelocity(-15)).andThen(intake.outtakePiece())
    // ,arm.setArmDegree(ArmPositions.Amp)).andThen(intake.stop());
    return Commands.deadline(Commands.waitSeconds(.001)
    .andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
    ,arm.setArmDegree(ArmPositions.Amp), intake.setVelocity(10))
    .andThen(Commands.waitSeconds(1))
    .andThen(Commands.deadline(Commands.waitSeconds(.2),intake.setVelocity(-18.25)/* 
    ,(arm.setArmDegree(ArmPositions.AmpMove))*/));
  }

  public Command shoot() {
    return arm.getCurrentArmState() == ArmState.Speaker ? loadAndShoot() : ampShot();
  }
  public Command loadAndShoot(){
    return Commands.run(() -> shooter.setTargetFlywheelSpeed(100));
    // return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed())),
    // Commands.run(() -> shooter.setTargetFlywheelSpeed(80)))
    // .andThen(intake.outtakePiece())
    // .andThen(Commands.parallel(intake.stop()));
  }
  // public Command intakeFromShooter(){

  //   return Commands.deadline(Commands.runOnce(()->shooter.setTargetFlywheelSpeed(-15))
  //   .andThen(intake.intakePiece())
  //   .andThen(Commands.runOnce(()->shooter.setTargetFlywheelSpeed(15)))
  //   ,arm.setArmDegree(ArmPositions.Load));
  // }
  public Command speakerFromIntake(){
    return Commands.deadline(Commands.waitSeconds(2)
    ,Commands.waitSeconds(1).alongWith(intake.setVelocity(5)).andThen(intake.setVelocity(50))
    ,arm.setArmDegree(ArmPositions.Amp)).andThen(intake.stop());
  }
  public Command overrideAll(){ // I dont think this works, but some sort of thing like this should be implimented
    return Commands.runOnce(()->{
      arm.setArmDegree(ArmPositions.Load);
      intake.stop();
      shooter.setTargetFlywheelSpeed(40);// Default speed
    });
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController thetaController = new PIDController(.0001,0,10);
  public Command alignToAmp(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    // thetaController.enableContinuousInput(-, Math.PI);
    thetaController.reset();

    // thetaController.setSetpoint(90);

    DoubleSupplier rotationalVelocity = () -> -thetaController.calculate(drivetrain.getYaw().getDegrees(), 0);
    
    // System.out.println(rotationalVelocity);
    return drivetrain.applyRequest(() -> drive.withVelocityX(-xAxis.getAsDouble()) 
      .withVelocityY(-yAxis.getAsDouble())
      .withRotationalRate(rotationalVelocity.getAsDouble()));
  }

  public Command alignToSpeaker(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    thetaController.reset();
    

    DoubleSupplier rotationalVelocity = () -> -thetaController.calculate(drivetrain.getYaw().getDegrees(), 
                                      drivetrain.getAngleFromSpeaker().getDegrees());
  
    
    return drivetrain.applyRequest(() -> drive.withVelocityX(-xAxis.getAsDouble()) 
      .withVelocityY(-yAxis.getAsDouble())
      .withRotationalRate(rotationalVelocity.getAsDouble()/12));
  }
}
