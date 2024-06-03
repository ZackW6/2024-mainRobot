// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.IntakeConstants;

/** Add your docs here. */
public class FlyWheelSim {

    private final FlywheelSim flywheelSim;
    private final TalonFXSimState motorSim;

    private String name;
    private double currentRad = 0;
    private double currentTime = 0;

    public FlyWheelSim(TalonFX motor, FlywheelSim flywheelSim){
        this.flywheelSim = flywheelSim;
        this.motorSim = motor.getSimState();
    }

    private Mechanism2d mech2d;
    private MechanismRoot2d pivot;

    private MechanismRoot2d directionPivot;
    private MechanismLigament2d directionLine;
    
    private MechanismLigament2d[] spokes;


    public void addSimImage(String name, int numOfSpokes){
        this.name = name;
        mech2d = new Mechanism2d(60, 60);
        pivot = mech2d.getRoot(name+" Pivot", 30, 30);
        spokes = new MechanismLigament2d[numOfSpokes];

        for (int i = 0; i < spokes.length;i++){
            spokes[i] = pivot.append(
            new MechanismLigament2d(
                name+" Spoke "+i,
                10,
                Units.radiansToDegrees(currentRad)+(360/spokes.length)*i,
                6,
                new Color8Bit(Color.kPurple)));
        }
        SmartDashboard.putData(name, mech2d);
    }


    private double lastVelocity = 0;
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        flywheelSim.setInput(motorSim.getMotorVoltage());
        // // Next, we update it. The standard loop time is 20ms.
        flywheelSim.update(0.02);
        //Makes the rest of the robot react based on this usage of Voltage
        motorSim.setRotorAcceleration((flywheelSim.getAngularVelocityRadPerSec()-lastVelocity)/(Timer.getFPGATimestamp()-currentTime));
        lastVelocity = flywheelSim.getAngularVelocityRadPerSec();
        motorSim.setRawRotorPosition(Units.radiansToRotations(currentRad));
        motorSim.setRotorVelocity(Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec()));
        

        // Update the Mechanism Arm angle based on the simulated arm angle
        currentRad+=(flywheelSim.getAngularVelocityRadPerSec()*(Timer.getFPGATimestamp()-currentTime));
        currentTime = Timer.getFPGATimestamp();
        if (spokes[0] != null){
            for (int i = 0; i<spokes.length;i++){
                spokes[i].setAngle(Units.radiansToDegrees(currentRad)+(360/spokes.length)*i);
            }
        }
        try {
            if (flywheelSim.getAngularVelocityRadPerSec()>0){
                directionLine.setColor(new Color8Bit(Math.min((int)flywheelSim.getAngularVelocityRadPerSec()/3,255),0,0));
            }else{
                directionLine.setColor(new Color8Bit(0,0,Math.min(Math.abs((int)flywheelSim.getAngularVelocityRadPerSec()/3),255)));
            }
        } catch (Exception e) {
            return;
        }
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
        
    }
    public void addDirectionColor(){
        directionPivot = mech2d.getRoot(name+" Direction Pivot", 0, 60);
        directionLine = directionPivot.append(
            new MechanismLigament2d(
                name+" Direction Line",
                60,
                Units.radiansToDegrees(0),
                10,
                new Color8Bit(Color.kRed)));  
    }
}
