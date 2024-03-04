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


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;


import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


import static edu.wpi.first.units.Units.Volts;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armMotor;
  private final CANcoder encoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  // private GenericEntry armFeedForward;
  // private GenericEntry armP;
  // private GenericEntry armI;
  // private GenericEntry armD;
  // private GenericEntry angleTarget;
  //angle, velocity, current, temp
  private GenericEntry armCurrent;
  private GenericEntry armVelocity;
  private GenericEntry armTemp;
  private GenericEntry armAngle;
  private final VoltageOut m_sysidControl = new VoltageOut(0);
  private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
  private SysIdRoutine m_SysIdRoutine;
  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  private ArmState currentArmState = ArmState.Speaker; 
  public enum ArmState {
    Speaker,
    Amp,
    Intake,
    AmpMove
  } 


  public Arm() {

    armMotor = new TalonFX(ArmConstants.armMotorID);
    encoder = new CANcoder(ArmConstants.CANCoderID);
    //resetToAbsolute();
    configMotor();
    // angleTarget = tab
    //   .add("angleTarget", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min",-20,"max",180))
    //   .getEntry();
    // armFeedForward = tab
    //   .add("armFeedForward", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min",0,"max",300)) //DONT NEED SINCE CTRE, BUT KEEPING JUST IN CASE
    //   .getEntry();
    // armP = tab
    //   .add("armP", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min",0,"max",200))
    //   .getEntry();
    // armI = tab
    //   .add("armI", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min",0,"max",10))
    //   .getEntry();
    // armD = tab
    //   .add("armD", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min",0,"max",10))
    //   .getEntry();
    tab.addNumber("armCurrent", ()->armMotor.getTorqueCurrent().getValueAsDouble());

    tab.addNumber("armVelocity", ()->armMotor.getVelocity().getValueAsDouble());

    tab.addNumber("armTemp", ()->armMotor.getDeviceTemp().getValueAsDouble());

    tab.addNumber("armAngle", ()->getArmDegrees());
    // m_SysIdRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,         // Default ramp rate is acceptable
    //             Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
    //             null,          // Default timeout is acceptable
    //                                    // Log state with Phoenix SignalLogger class
    //             (state)->SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //             (Measure<Voltage> volts)-> armMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
    //             null,
    //             this));
    //     setName("Arm");

    //     // TalonFXConfiguration cfg = new TalonFXConfiguration();
    //     // leftShooterMotor.getConfigurator().apply(cfg);

    //     /* Speed up signals for better charaterization data */
    //     BaseStatusSignal.setUpdateFrequencyForAll(250,
    //         armMotor.getPosition(),
    //         armMotor.getVelocity(),
    //         armMotor.getMotorVoltage());

    //     /* Optimize out the other signals, since they're not particularly helpful for us */
    //     armMotor.optimizeBusUtilization();
    //     SignalLogger.setPath("/media/sda1/");//HERE
    //     SignalLogger.start();
    
  }
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return m_SysIdRoutine.quasistatic(direction);
  // }
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return m_SysIdRoutine.dynamic(direction);
  // }

  MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(.47);

  private void setArmDegree(ArmState armPosition, boolean isActive){
   double rotSet;
      switch (armPosition){
        case Intake:
          rotSet=-0.098;
          break;
        case Speaker:
          rotSet=.47;
          break;
        case Amp:
          rotSet=.265;
          break;
        case AmpMove:
          rotSet= .262;
          break;
        default:
          rotSet=.4422222222;
          break;
      }
      armMotor.setControl(m_request.withPosition(rotSet).withSlot(0));
  }
  public ArmState getCurrentArmState(){
    return currentArmState;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ArmPositions defaultState = getCurrentArmState() == ArmState.Speaker ? ArmPositions.Load : ArmPositions.Amp;
    // System.out.println(getCurrentArmState());
    // setDefaultCommand(setArmDegree(ArmPositions.Amp));
    
    if (currentArmState == ArmState.Speaker) {
      lastMainState = ArmState.Speaker;
      setArmDegree(ArmState.Speaker, true);
    } else if (currentArmState == ArmState.Amp){
      lastMainState = ArmState.Amp;
      setArmDegree(ArmState.Amp, true);
    }else if (currentArmState == ArmState.Intake){
      setArmDegree(ArmState.Intake, true);
    }else{
      setArmDegree(ArmState.AmpMove, true);
    }
  }

  private ArmState lastMainState = ArmState.Speaker;

  public ArmState lastMainState(){
    return lastMainState;
  }


  private void configMotor(){
    talonFXConfigs = new TalonFXConfiguration();
    CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
    // //Set to factory default
    encoder.getConfigurator().apply(new CANcoderConfiguration());
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    
    
    canCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfigs.MagnetSensor.MagnetOffset = ArmConstants.angleOffset.getRotations();
    
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
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.cruiseVelocity; // Unlimited cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.maxAcceleration;
    motionMagicConfigs.MotionMagicJerk = ArmConstants.jerk;
    // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
    /* Current Limits */
    m_currentLimits.SupplyCurrentLimit = ArmConstants.armCurrentLimit; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = ArmConstants.armSupplyCurrentThreshold; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = ArmConstants.armCurrentThresholdTime; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = ArmConstants.armEnableCurrentLimit; // And enable it
    m_currentLimits.StatorCurrentLimit = ArmConstants.armStatorCurrentLimit; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = ArmConstants.armStatorCurrentLimitEnable; // And enable it

    talonFXConfigs.CurrentLimits = m_currentLimits;

    armMotor.getConfigurator().apply(talonFXConfigs);
  }
  
  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
  }

  public double getArmDegrees() {
    //return Rotation2d.fromRotations(getCANcoder().getRotations());
    return getCANcoder().getDegrees();
  }

  public double getRotationTarget() {
    return armMotor.getClosedLoopReference().getValueAsDouble();
  }

  public Boolean isArmInAmpState() {
      return currentArmState == ArmState.Amp;
  }

  public void setCurrentArmState(ArmState armState) {
    // System.out.println(armState);
    currentArmState = armState;
  }
  public boolean isArmAtAngle(){
    if (armMotor.getClosedLoopReference().getValueAsDouble()<=getRotationTarget()+.03 || armMotor.getClosedLoopReference().getValueAsDouble()<=getRotationTarget()-.03){
      // System.out.println("Arm is at pos");

      return true;
    } 
    return false;
  }
}