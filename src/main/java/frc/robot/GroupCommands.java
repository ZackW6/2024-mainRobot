// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  //THESE ARE JUST SOME, I IMAGINE WE WILL NEED VERY DIFFERENT, BUT THESE CAN ALSO BE A PROOF OF CONCEPT
  //AUTO COMMANDS
  // public Command intakeThenSpeaker(){
  //   return Commands.sequence(Commands.deadline(intake.intakePiece(),arm.setArmDegree(ArmPositions.Intake))
  //   ,arm.setArmDegree(ArmPositions.Load),shooter.shootVelocity(10, 10));
  // }
  public Command intake(){
    return Commands.deadline(intake.intakePiece(),arm.setArmDegree(ArmPositions.Intake));
  }
  // public Command armUp(){
  //   return arm.setArmDegree(ArmPositions.Load);
  // }
  public Command loadAndShoot(){
    // return Commands.sequence(shooter.shootVelocity(100, 100)
    // ,Commands.waitSeconds(0.2)/*Wait time to speed up*/,intake.outtakePiece()
    // ,Commands.waitSeconds(1),shooter.rest(),intake.stop());
    return Commands.deadline(Commands.waitSeconds(4)
    ,Commands.runOnce(()->shooter.setTargetFlywheelSpeed(100)))
    .andThen(intake.outtakePiece())
    .andThen(Commands.parallel(intake.stop()
    , Commands.runOnce(()->shooter.setTargetFlywheelSpeed(10))));
  }
  // public Command armDown(){
  //   return arm.setArmDegree(ArmPositions.Intake);
  // }

  // public Command loadAndShootBackDown(){
  //   return Commands.sequence(shooter.shootVelocity(100, 100)
  //   ,Commands.waitSeconds(0.2)/*Wait time to speed up*/,intake.outtakePiece()
  //   ,Commands.waitSeconds(1),shooter.rest(),intake.stop(),armDown());
  // }
  

  public Command intakeThenAmp(){
    return Commands.sequence(Commands.deadline(intake.intakePiece()));
  } 
  // public Command outtake(){
  //   return intake.outtakePiece();
  // } 
}
