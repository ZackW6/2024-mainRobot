package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase{
    private static final double FLYWHEEL_ALLOWABLE_ERROR = Units.rotationsPerMinuteToRadiansPerSecond(75);
    private static final double FLYWHEEL_GEAR_REDUCTION = 1;

    private static final double FLYWHEEL_VELOCITY_CONSTANT = 0.028;
    private static final double FLYWHEEL_ACCELERATION_CONSTANT = 0.0030108;
    private static final double FLYWHEEL_SENSOR_POSITION_COEFFICIENT = (FLYWHEEL_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    private static final double FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT = FLYWHEEL_SENSOR_POSITION_COEFFICIENT * 10.0;
    public static final double FLYWHEEL_IDLE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    private final TalonFX leftShooterMotor;
    private final TalonFX  rightShooterMotor;

    private boolean flywheelDisabled = false;
    private double targetFlywheelSpeed;
    private double shootingOffset = 0.0;

    // private final NetworkTableEntry shooterRPMOffsetEntry;


    private final VoltageOut m_sysidControl = new VoltageOut(0);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    private SysIdRoutine m_SysIdRoutine;

    public Shooter(){

        leftShooterMotor = new TalonFX(ShooterConstants.leftShooterMotorID);
        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterMotorID);
        // TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        // leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        // leftShooterMotor.setInverted(true);
        // rightShooterMotor.setInverted(false);
        configMotors();

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Left Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getLeftFlywheelVelocity()));
        shuffleboardTab.addNumber("Right Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getRightFlywheelVelocity()));
        shuffleboardTab.addNumber("Target Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getTargetFlywheelSpeed()));
        shuffleboardTab.addBoolean("Is Left Flywheel at Speed", this::isLeftFlywheelAtTargetSpeed);
        shuffleboardTab.addBoolean("Is Right Flywheel at Speed", this::isLeftFlywheelAtTargetSpeed);
        // shooterRPMOffsetEntry = Shuffleboard.getTab("Driver")
        //         .add("Shooting Offset", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", -250.0, "max", 250.0, "Block increment", 25.0))
        //         .withPosition(2, 1)
        //         .getEntry();
    
        // leftSpeed = Shuffleboard.getTab("Shooter")
        //     .add("LeftSpeed", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .withProperties(Map.of("min",-100,"max",100))
        //     .getEntry();

        // rightSpeed = Shuffleboard.getTab("Shooter")
        //     .add("RightSpeed", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .withProperties(Map.of("min",-100,"max",100))
        //     .getEntry();

        // m_SysIdRoutine = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         null, Volts.of(4), null, // Use default config
        //         (state) -> SignalLogger.writeString("SysIdTestState", state.toString())
        //     ),
        //     new SysIdRoutine.Mechanism(
        //         (Measure<Voltage> volts) -> leftShooterMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
        //         null, // No log consumer, since data is recorded by AdvantageKit
        //         this
        //     ) //I WANT TO TRY THIS PROBABLY, I WONDER IF IT AUTO DOES VOLTS, POS, AND VELOCITY?////////////////////////////////////////
        // );

        // The methods below return Command objects
        // m_SysIdRoutine =
        // new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         null,         // Default ramp rate is acceptable
        //         Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
        //         null,       // Default timeout is acceptable
        //                     // Log state with Phoenix SignalLogger class
        //         (state)->SignalLogger.writeString("state", state.toString())),
        //     new SysIdRoutine.Mechanism(
        //         (Measure<Voltage> volts)-> leftShooterMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
        //         (motor) -> {
        //             SignalLogger.writeDouble("Voltage", leftShooterMotor.getSupplyVoltage().getValue());
        //             SignalLogger.writeDouble("Position", leftShooterMotor.getRotorPosition().getValueAsDouble());
        //             SignalLogger.writeDouble("Velocity", leftShooterMotor.getRotorVelocity().getValueAsDouble());
        //             // SignalLogger.writeDouble("Voltage", leftShooterMotor.getSupplyVoltage().getValueAsDouble()  );
        //             // SignalLogger.writeDouble("Position", leftShooterMotor.getPosition().getValueAsDouble());
        //             // SignalLogger.writeDouble("Velocity", leftShooterMotor.getVelocity().getValueAsDouble());
        //         },
        //         this));
        // SignalLogger.setPath("/home/lvuser/logs/");
        // SignalLogger.start();

        // setName("Shooter");
        // // leftShooterMotor.getA
        // // TalonFXConfiguration cfg = new TalonFXConfiguration();
        // // leftShooterMotor.getConfigurator().apply(cfg);

        // /* Speed up signals for better charaterization data */
        // BaseStatusSignal.setUpdateFrequencyForAll(250,
        //     leftShooterMotor.getPosition(),
        //     leftShooterMotor.getVelocity(),
        //     leftShooterMotor.getMotorVoltage());

        // /* Optimize out the other signals, since they're not particularly helpful for us */
        // leftShooterMotor.optimizeBusUtilization();
        // SignalLogger.setPath("/home/lvuser/logs/");
        // SignalLogger.start();
    }    
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_SysIdRoutine.quasistatic(direction);
    // }
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_SysIdRoutine.dynamic(direction);
    // }
    
    @Override
    public void periodic() {
        if (flywheelDisabled) {
            leftShooterMotor.stopMotor();
            rightShooterMotor.stopMotor();
        } else {
            // double feedForward = FLYWHEEL_VELOCITY_CONSTANT * targetFlywheelSpeed / 12.0;

            leftShooterMotor.setControl(velocityRequest.withVelocity(targetFlywheelSpeed));//Shouldnt need/ since tuned kV* .withFeedForward(feedForward));*/
            rightShooterMotor.setControl(velocityRequest.withVelocity(targetFlywheelSpeed));//Shouldnt need/ since tuned kV* .withFeedForward(feedForward));*/;
        }

        // shootingOffset = shooterRPMOffsetEntry.getDouble(0.0);
    }

    @Deprecated
    public Command shootVelocity(double Rvelocity, double Lvelocity){
        return this.run(() -> {
            leftShooterMotor.setControl(velocityRequest.withVelocity(Lvelocity).withAcceleration(1000));
            rightShooterMotor.setControl(velocityRequest.withVelocity(Rvelocity).withAcceleration(1000));
        });
    }

    @Deprecated
    public Command stopMotors(){
        return this.runOnce(() -> {
            leftShooterMotor.stopMotor();
            rightShooterMotor.stopMotor();
        });
    }

    public void setTargetFlywheelSpeed(double targetFlywheelSpeed) {
        this.targetFlywheelSpeed = targetFlywheelSpeed;
    }

    public double getTargetFlywheelSpeed() {
        return targetFlywheelSpeed;
    }

    public double getLeftFlywheelVelocity(){
        return leftShooterMotor.getVelocity().getValueAsDouble();
    }

    public double getRightFlywheelVelocity(){
        return rightShooterMotor.getVelocity().getValueAsDouble();
    }

    public boolean isLeftFlywheelAtTargetSpeed() {
        return Math.abs(getLeftFlywheelVelocity() - targetFlywheelSpeed) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public boolean isRightFlywheelAtTargetSpeed() {
        return Math.abs(getRightFlywheelVelocity() - targetFlywheelSpeed) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public void disableFlywheel() {
        flywheelDisabled = true;
    }

    public void enableFlywheel() {
        flywheelDisabled = false;
    }

    private void configMotors(){
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterConstants.kS;
        slot0Configs.kV = ShooterConstants.kV;
        slot0Configs.kA = ShooterConstants.kA;
        slot0Configs.kP = ShooterConstants.kP;
        slot0Configs.kI = ShooterConstants.kI;
        slot0Configs.kD = ShooterConstants.kD;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit; // Limit to 1 amps
        currentLimits.SupplyCurrentThreshold = ShooterConstants.shooterCurrentThreshold; // If we exceed 4 amps
        currentLimits.SupplyTimeThreshold = ShooterConstants.shooterCurrentThresholdTime; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit; // And enable it

        currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = true; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;
        
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);
        leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        leftShooterMotor.setInverted(true);
    }
}