package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.simulationUtil.FlyWheelSim;


public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor;

    private final DigitalInput limitSwicth1;
    private final DigitalInput limitSwicth2;

    private final double intakeSpeed = 23.0;//In rps
    private final double outtakeSpeed = -60.0;

    private final FlywheelSim fSim = new FlywheelSim(DCMotor.getFalcon500(1), IntakeConstants.GEARING, IntakeConstants.MOMENT_OF_INERTIA);
    private final FlyWheelSim flywheelSim;

    public Intake(){
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        limitSwicth1 = new DigitalInput(IntakeConstants.LIMIT_SWITCH_ID_1);
        limitSwicth2 = new DigitalInput(IntakeConstants.LIMIT_SWITCH_ID_2);

        flywheelSim = new FlyWheelSim(intakeMotor, fSim);
        flywheelSim.addSimImage("Intake Sim",4);
        flywheelSim.addDirectionColor();

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
        shuffleboardTab.addNumber("Intake Speed",
                () -> getVelocity());
        shuffleboardTab.addNumber("Intake Target Speed",
                () -> getTargetVelocity());

        configMotors();
    }
    private final MotionMagicVelocityTorqueCurrentFOC torqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);

    public Command intakePiece(){
        if (Robot.isSimulation()){
            return setVelocity(intakeSpeed).andThen(Commands.waitSeconds(2000)).until(() -> getLimitSwitch())
                .finallyDo(()->stop());
        }
        return setVelocity(intakeSpeed).until(() -> getLimitSwitch())
            .finallyDo(()->stop());
    }
    public Command outtakePiece(){
        return Commands.deadline((Commands.waitSeconds(2)),setVelocity(outtakeSpeed)).andThen(stop());
    }

    public Command setVelocity(double rps){
        return this.runOnce(() -> intakeMotor.setControl(torqueCurrentFOC.withVelocity(rps)));
    }

    public Command stop(){
        return this.runOnce(()->intakeMotor.stopMotor()).alongWith(Commands.runOnce(()->intakeMotor.setControl(torqueCurrentFOC.withVelocity(0))));
    }

    public boolean getLimitSwitch(){
        return (!limitSwicth1.get() || !limitSwicth2.get());
    }

    public double getVelocity(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetVelocity(){
        return intakeMotor.getClosedLoopReference().getValueAsDouble();
    }

    public boolean isPiecePresent(){
        return getLimitSwitch();
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()){
            flywheelSim.simulationPeriodic();
        }
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
    
}
