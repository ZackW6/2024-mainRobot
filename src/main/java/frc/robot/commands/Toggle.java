// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Toggle extends Command {
  boolean state = false;
  Runnable runnerOne;
  Runnable runnerTwo;
  /** Creates a new Toggle. */
  public Toggle(Runnable optionOne, Runnable optionTwo, Subsystem... requirements) {
    this.runnerOne = optionOne;
    this.runnerTwo = optionTwo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = !state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state){
      runnerOne.run();
    }else{
      runnerTwo.run();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
