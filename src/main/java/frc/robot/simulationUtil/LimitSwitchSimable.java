// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/** Add your docs here. */
public class LimitSwitchSimable {
    private boolean bool;
    private boolean inverse;
    private final DigitalInput limitSwitch;
    public LimitSwitchSimable(int channel){
        limitSwitch = new DigitalInput(channel);
    }
    public void setSimInverse(boolean bool){
        inverse = bool;
    }
    public boolean isSimInverse(){
        return inverse;
    }
    public void setSimValue(boolean bool){
        this.bool = bool;
    }
    public boolean get(){
        if (!Robot.isSimulation()){
            return limitSwitch.get();
        }
        if (inverse){
            return !bool;
        }
        return bool;
    }
    public int getChannel(){
        return limitSwitch.getChannel();
    }
    public Command waitThenSwicthSim(double sec){
        return Commands.waitSeconds(sec).andThen(()->setSimValue(!bool));
    }

}
