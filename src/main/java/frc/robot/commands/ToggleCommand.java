// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ToggleCommand extends InstantCommand {
    private final Command command;
    private boolean isRunning = false;

    public ToggleCommand(Command command) {
        this.command = command;
        addRequirements(command.getRequirements().toArray(new Subsystem[0])); // Ensure requirements are transferred
    }

    @Override
    public void initialize() {
        if (isRunning) {
            command.cancel(); // If the command is running, cancel it
            isRunning = false;
        } else {
            command.schedule(); // If the command is not running, schedule it
            isRunning = true;
        }
    }
}