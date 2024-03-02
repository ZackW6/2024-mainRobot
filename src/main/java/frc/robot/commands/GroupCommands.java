// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPositions;

public class GroupCommands  {
  private Arm arm;
  private Shooter shooter;
  private Intake intake;
  private Candle candle;
  private boolean bool = true;
  public GroupCommands(Arm arm, Shooter shooter, Intake intake, Candle candle){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    this.candle = candle;
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
    .andThen(Commands.deadline(Commands.waitSeconds(.2),intake.setVelocity(-18)/* 
    ,(arm.setArmDegree(ArmPositions.AmpMove))*/));
  }

  public Command loadAndShoot(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(()->shooter.isLeftFlywheelAtTargetSpeed()))
    ,Commands.runOnce(()->shooter.setTargetFlywheelSpeed(100)))
    .andThen(intake.outtakePiece())
    .andThen(Commands.parallel(intake.stop()
    , Commands.runOnce(()->shooter.setTargetFlywheelSpeed(15))));
  }
  public Command changeArmDefault(){
    return Commands.runOnce(()->{
      if (bool==true){
        arm.setDefaultCommand(arm.setArmDegree(ArmPositions.Amp));
        bool=false;
      }else{
        arm.setDefaultCommand(arm.setArmDegree(ArmPositions.Load));
        bool=true;
      }
    });
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
}
