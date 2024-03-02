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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPositions;

public class GroupCommands  {
  private Arm arm;
  private Shooter shooter;
  private Intake intake;

  public GroupCommands(Arm arm, Shooter shooter, Intake intake){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
  }
  public Command intake(){
    return Commands.deadline(intake.intakePiece(),arm.setArmDegree(ArmPositions.Intake))
    .andThen(Commands.deadline(Commands.waitSeconds(1),intake.setOutput(10)))
    .andThen(intake.stop());
  }
  public Command ampShot(){
    return Commands.deadline(Commands.waitSeconds(2)
    ,Commands.waitSeconds(1).alongWith(intake.setOutput(10)).andThen(intake.outtakePiece())
    ,arm.setArmDegree(ArmPositions.Amp));
  }
  public Command loadAndShoot(){
    return Commands.deadline(Commands.waitSeconds(4)
    ,Commands.runOnce(()->shooter.setTargetFlywheelSpeed(100)))
    .andThen(intake.outtakePiece())
    .andThen(Commands.parallel(intake.stop()
    , Commands.runOnce(()->shooter.setTargetFlywheelSpeed(10))));
  }
  public Command intakeFromShooter(){
    return Commands.deadline(Commands.runOnce(()->shooter.setTargetFlywheelSpeed(-5))
    .andThen(intake.intakePiece())
    .andThen(Commands.runOnce(()->shooter.setTargetFlywheelSpeed(10)))
    ,arm.setArmDegree(ArmPositions.Load));
  }
  public Command overrideAll(){ // I dont think this works, but some sort of thing like this should be implimented
    return Commands.runOnce(()->{
      arm.setArmDegree(ArmPositions.Load);
      intake.stop();
      shooter.setTargetFlywheelSpeed(10);// Default speed
    });
  }
}
