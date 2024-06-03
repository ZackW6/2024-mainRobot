// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ArmConstants;

/** Add your docs here. */
public class ArmSim{
    private boolean includeCANCoder = false;
    private CANcoderSimState encoderSim;
    private TalonFXSimState motorSim;
    private double offset;

    private final SingleJointedArmSim armSim;

    private Mechanism2d mech2d;
    private MechanismRoot2d armPivot;
    private MechanismLigament2d armTower;
    private MechanismLigament2d arm;
    public ArmSim(TalonFX motor, SingleJointedArmSim armSim){
        this.armSim = armSim;
        motorSim = motor.getSimState();
    }

    public void addSimImage(String name){
        mech2d = new Mechanism2d(60, 60);
        armPivot = mech2d.getRoot(name+ " ArmPivot", 30, 30);
        armTower = armPivot.append(new MechanismLigament2d(name+" ArmTower", 30, -90));
        arm = armPivot.append(
          new MechanismLigament2d(
              name + " Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
        armTower.setColor(new Color8Bit(Color.kBlue));
        SmartDashboard.putData(name, mech2d);
    }

    /**
     * offset will likely be different by .25 incriment rotations compared to correct offset.
     * @param encoder
     * @param orientation
     * @param offset
     */
    public void configureCANCoder(CANcoder encoder, ChassisReference orientation, double offset){
        includeCANCoder = true;
        encoderSim = encoder.getSimState();
        encoderSim.Orientation = orientation;
        this.offset = offset;
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)    

        armSim.setInput(motorSim.getMotorVoltage());

        armSim.update(0.02);

        if (includeCANCoder){
            encoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads())-offset);
            encoderSim.setVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec()));
        }
        motorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads())-offset);
        motorSim.setRotorVelocity(Units.radiansToRotations(Units.radiansToRotations(armSim.getVelocityRadPerSec())));
        // encoder.setPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
        // System.out.println(encoder.getPosition().getValue());
        // // Finally, we set our simulated encoder's readings and simulated battery voltage
        // // SimBattery estimates loaded battery voltages

        //Makes the rest of the robot react based on this usage of Voltage
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
}
