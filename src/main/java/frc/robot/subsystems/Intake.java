package frc.robot.subsystems;



import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;


public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor;
    private final DigitalInput limitSwicth1;
    private final DigitalInput limitSwicth2;
    private double speed = 0;
    double intakeSpeed = 23.0;//In rps
    double outtakeSpeed = -60.0;

    public Intake(){
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        limitSwicth1 = new DigitalInput(IntakeConstants.LIMIT_SWITCH_ID_1);
        limitSwicth2 = new DigitalInput(IntakeConstants.LIMIT_SWITCH_ID_2);
        configMotors();
    }
    MotionMagicVelocityTorqueCurrentFOC torqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
    public Command intakePiece(){
        return setVelocity(intakeSpeed).until(() -> getLimitSwitch())
            .andThen(stop());
    }
    public Command outtakePiece(){
        return Commands.deadline((Commands.waitSeconds(2)),setVelocity(outtakeSpeed)).andThen(stop());
    }

    public Command setVelocity(double rps){
        speed=rps;
        return this.run(() -> intakeMotor.setControl(torqueCurrentFOC.withVelocity(rps))).alongWith(Commands.runOnce(()->speed=rps));
    }

    public Command stop(){
        return this.runOnce(()->intakeMotor.stopMotor());
    }

    public boolean getLimitSwitch(){
        return (!limitSwicth1.get() || !limitSwicth2.get());
    }

    @Override
    public void periodic() {
        // System.out.println(speed+ " INTAKE SPEED");

        // System.out.println("GET LIMIT SWITCH: "+getLimitSwitch());
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
        motionMagicConfigs.MotionMagicAcceleration = 10000; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentThreshold = IntakeConstants.INTAKE_CURRENT_THRESHOLD ; // If we exceed 4 amps
        currentLimits.SupplyTimeThreshold = IntakeConstants.INTAKE_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = IntakeConstants.INTAKE_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
    }
    public boolean isPiecePresent(){
        return getLimitSwitch();
    }
}
