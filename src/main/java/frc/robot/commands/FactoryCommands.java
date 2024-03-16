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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;

public class FactoryCommands extends SubsystemBase{
  private Arm arm;
  private Shooter shooter;
  private Intake intake;
  private Candle candle;
  private CommandSwerveDrivetrain drivetrain;
  public FactoryCommands(Arm arm, Shooter shooter, Intake intake, Candle candle, CommandSwerveDrivetrain drivetrain){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    this.candle = candle;
    this.drivetrain = drivetrain;
  }

  public Command shoot() {
    return Commands.either(ampShot(), loadAndShoot(), ()->arm.isArmInAmpState()) ;
  }

  public Command intake() {
    return intakeMain();
    // return Commands.either(resetAll(), intake(), ()->arm.isArmInIntakeState()) ;
  }

  public Command loadAndShoot(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed()))
      ,Commands.run(() -> {
        double dist = drivetrain.getDistanceFromSpeakerMeters();
        if (dist<=2 || dist ==-1){
          shooter.setTargetFlywheelSpeed(85,85);
        }else if(dist<=3){
          shooter.setTargetFlywheelSpeed(75,75);
        }else if(dist>3){
          shooter.setTargetFlywheelSpeed(65,65);
        }
      }), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }
  
  public Command ampShot(){
    return Commands.deadline(Commands.waitSeconds(.5)
      ,Commands.waitSeconds(0.003).andThen(Commands.runOnce(()->arm.setCurrentArmState(ArmState.AmpMove)))
      ,intake.setVelocity(-19.25/*-19.5*/)).andThen(switchModes());
  } 

  public Command intakeMain(){
    return Commands.deadline(intake.intakePiece(),Commands.runOnce(()->arm.setCurrentArmState(ArmState.Intake)),Commands.runOnce(()->shooter.enableDefault()))
      .andThen(candle.pickUpLights())
      .andThen(Commands.deadline(Commands.waitSeconds(.5).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(20)
      ,Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState()))))
      .andThen(intake.stop())
      .andThen(candle.idleLED());
  }

  public Command intakeMainAuto(){
    return Commands.race(Commands.waitSeconds(2.5)
      ,Commands.deadline(intake.intakePiece(),Commands.runOnce(()->arm.setCurrentArmState(ArmState.Intake)),Commands.runOnce(()->shooter.enableDefault()))
      .andThen(candle.pickUpLights()))
      .andThen(Commands.deadline(Commands.waitSeconds(.5).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)
      ,Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState()))))
      .andThen(intake.stop())
      .andThen(candle.idleLED());
  }

  public Command resetAll(){
    return Commands.runOnce(()->shooter.enableDefault())
      .andThen(intake.stop())
      .andThen(Commands.runOnce(()->arm.setCurrentArmState(ArmState.Speaker)));
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController thetaControllerAmp = new PIDController(12.2,0,.5);//(12.2,1.1,.4);
  public Command alignToAmp(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    // thetaController.enableContinuousInput(-, Math.PI);
    thetaControllerAmp.reset();
    // thetaController.setSetpoint(90);
    var alliance = DriverStation.getAlliance();
    if(!alliance.isPresent()) {
      return Commands.none();
    }
    DoubleSupplier rotationalVelocity = () -> {
      if(DriverStation.Alliance.Blue.equals(alliance.get())){
        return thetaControllerAmp.calculate(correctYaw((drivetrain.getPose().getRotation().getDegrees()-90)%360,270), 270);
      }else{
        return thetaControllerAmp.calculate(correctYaw((drivetrain.getPose().getRotation().getDegrees()-90)%360,0), 0);
      }
    };
    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) //was -xAxis, but in sim is this
      .withVelocityY(yAxis.getAsDouble())//was -yAxis, but in sim is this
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }
  private double correctYaw(double x, double setpoint){
    if (x>180+setpoint){
      x-=360;
    }else if(x<-180+setpoint){
      x+=360;
    }
    return x;
  }
  private final PIDController thetaControllerSpeaker = new PIDController(7,0,0);
  private final PIDController distanceYController = new PIDController(2, 0, 0);
  public Command alignToSpeaker(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    thetaControllerSpeaker.reset();
    

    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromSpeaker().getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromSpeaker().getDegrees(),0);//-7;
      }else{

        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromSpeaker().getDegrees(),0);//+7;
      }
      
    };
    DoubleSupplier distanceYVelocity = ()-> distanceYController.calculate(drivetrain.getDistanceFromSpeakerMeters(), 2);
    
    DoubleSupplier offset=()->{
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? (alliance.get().equals(Alliance.Red) ? 180 : 360) : 360;
    };
    // drivetrain.seedFieldRelative(180-drivetrain.getPose().getRotation().getDegrees());
    return drivetrain.applyRequest(() -> drive.withVelocityX(distanceYVelocity.getAsDouble()) 
      .withVelocityY(yAxis.getAsDouble())
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())))
      .alongWith(Commands.runOnce(()->{
      drivetrain.seedFieldRelative(offset.getAsDouble()-drivetrain.getPose().getRotation().getDegrees());
    }));
  }

  public Command switchModes(){
    return Commands.runOnce(() -> {
      if (arm.getCurrentArmState()!=ArmState.Intake){
        ArmState prevState = arm.getCurrentArmState();
        ArmState setState;
        if (prevState == ArmState.Speaker){
          shooter.disableFlywheel();
          setState=ArmState.Amp;
        }else{
          shooter.enableFlywheel();
          setState=ArmState.Speaker;
        }
        arm.setCurrentArmState(setState);
      }
      });
  }
}