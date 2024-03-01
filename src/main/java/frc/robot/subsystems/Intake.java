package frc.robot.subsystems;



import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;


public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor;
    private final DigitalInput m_limit;

    double intakeSpeed = 50.0;//In rps
    double outtakeSpeed = -50.0;

    public Intake(){
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
        m_limit = new DigitalInput(IntakeConstants.limitSwitchID);
        configMotors();
    }

    TorqueCurrentFOC velocityRequest = new TorqueCurrentFOC(0);
    public Command intakePiece(){
        return setVelocity(20).until(() -> m_limit.get())//BAD
            .andThen(Commands.deadline(Commands.waitSeconds(.3),setVelocity(10)))
            .andThen(stop());
    }

    public Command outtakePiece(){
        return Commands.deadline((new WaitCommand(2)),setVelocity(-35)).andThen(stop());
    }

    public Command setVelocity(double rps){
        return this.run(() -> intakeMotor.setControl(velocityRequest.withOutput(rps)));
    }

    public Command stop(){
        return this.runOnce(()->intakeMotor.stopMotor());
    }

    public boolean getLimitSwitch(){
        return m_limit.get();
    }

    @Override
    public void periodic() {
        System.out.println(m_limit.get());
    }
    

    private void configMotors(){
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kS = IntakeConstants.kS;
        // slot0Configs.kV = IntakeConstants.kV;
        // slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = IntakeConstants.intakeCurrentLimit; // Limit to 1 amps
        currentLimits.SupplyCurrentThreshold = IntakeConstants.intakeCurrentThreshold ; // If we exceed 4 amps
        currentLimits.SupplyTimeThreshold = IntakeConstants.intakeCurrentThresholdTime; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = IntakeConstants.intakeEnableCurrentLimit; // And enable it

        currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = true; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
    }
}
