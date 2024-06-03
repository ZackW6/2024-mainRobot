// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.simulationUtil.ArmSim;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Volts;


public class Arm extends SubsystemBase {
  private final TalonFX armMotor;
  private final CANcoder encoder;
  
  private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  public enum ArmState {
    Speaker(.47),
    Amp(.29),
    Intake(-.088),
    AmpMove(.21);
    private double rotation;
    ArmState(double rotation){
      this.rotation = rotation;
    }
    public double getRotation(){
      return rotation;
    }
  }

  private final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          ArmConstants.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH_METERS, ArmConstants.ARM_WEIGHT_KG),
          ArmConstants.ARM_LENGTH_METERS,
          ArmConstants.MIN_ARM_ANGLE_RAD,
          ArmConstants.MAX_ARM_ANGLE_RAD,
          true,
          Units.rotationsToRadians(.47)
          );
  private final ArmSim armSim;
  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  public Arm() {
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    encoder = new CANcoder(ArmConstants.CAN_CODER_ID);

    armSim = new ArmSim(armMotor, singleJointedArmSim);
    armSim.addSimImage("Arm Sim");
    armSim.configureCANCoder(encoder, ChassisReference.Clockwise_Positive,ArmConstants.ANGLE_OFFSET.getRotations()+.25);

    configMotor();

    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    tab.addNumber("armCurrent", ()->armMotor.getTorqueCurrent().getValueAsDouble());

    tab.addNumber("armVelocity", ()->armMotor.getVelocity().getValueAsDouble());

    tab.addNumber("armTemp", ()->armMotor.getDeviceTemp().getValueAsDouble());

    tab.addNumber("armAngle", ()->getArmDegrees());
  }

  MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(ArmState.Speaker.getRotation());

  public Command setArmRotation(ArmState armState){
    return this.run(()->armMotor.setControl(m_request.withPosition(armState.getRotation()).withSlot(0)));
  }

  public Command setArmDefault(ArmState state){
    return Commands.runOnce(()->setDefaultCommand(this.runOnce(()->armMotor.setControl(m_request.withPosition(state.getRotation()).withSlot(0)))));
  }

  private void configMotor(){
    talonFXConfigs = new TalonFXConfiguration();
    CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
    // //Set to factory default
    encoder.getConfigurator().apply(new CANcoderConfiguration());
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    
    
    canCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfigs.MagnetSensor.MagnetOffset = ArmConstants.ANGLE_OFFSET.getRotations();
    
    encoder.getConfigurator().apply(canCoderConfigs);

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ArmConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ArmConstants.kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ArmConstants.kI; // no output for integrated error
    slot0Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kG = ArmConstants.kG;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    Slot1Configs slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = ArmConstants.kS; // Add 0.25 V output to overcome static friction
    slot1Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot1Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot1Configs.kP = ArmConstants.kP; // A position error of 2.5 rotations results in 12 V output
    slot1Configs.kI = ArmConstants.kI; // no output for integrated error
    slot1Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in 0.1 V output
    slot1Configs.kG = ArmConstants.kG;
    slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//InvertedValue.Clockwise_Positive
    talonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
    talonFXConfigs.Feedback.RotorToSensorRatio = 32.0;
    
    /* Motion Magic Settings */
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY; // Unlimited cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = ArmConstants.JERK;
    // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
    /* Current Limits */
    m_currentLimits.SupplyCurrentLimit = ArmConstants.ARM_CURRENT_LIMIT; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = ArmConstants.ARM_SUPPLY_CURRENT_THRESHOLD; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = ArmConstants.ARM_CURRENT_THRESHOLD_TIME; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = ArmConstants.ARM_ENABLE_CURRENT_LIMIT; // And enable it
    m_currentLimits.StatorCurrentLimit = ArmConstants.ARM_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = ArmConstants.ARM_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

    talonFXConfigs.CurrentLimits = m_currentLimits;

    armMotor.getConfigurator().apply(talonFXConfigs);
  }
  
  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
  }

  public double getArmDegrees() {
    return getCANcoder().getDegrees();
  }

  public double getRotationTarget() {
    return armMotor.getClosedLoopReference().getValueAsDouble();
  }

  public boolean isArmAtAngle(){
    if (Units.degreesToRotations(getArmDegrees())<=getRotationTarget()+.03 && Units.degreesToRotations(getArmDegrees())>=getRotationTarget()-.03){
      return true;
    }
    return false;
  }

  public void periodic(){
    if (Robot.isSimulation()){
      armSim.simulationPeriodic();
    }
  }

  
}