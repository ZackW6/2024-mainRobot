// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ForCommand extends Command {
  private int i = 0;
  private final int num;
  private final Command command;
  /** Creates a new ForCommand. */
  public ForCommand(Command command, int numTimes) {
    this.addRequirements((Subsystem[])command.getRequirements().toArray());
    this.num = numTimes;
    this.command = command;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (i < num) {
      if (!command.isScheduled()) {
          command.schedule();
          i++;
      }
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i >= num && !command.isScheduled();
  }

  public int getIteration(){
    return i;
  }
}
