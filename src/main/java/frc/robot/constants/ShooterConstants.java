package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.util.MultiLinearInterpolator;

public class ShooterConstants {
    public static final double FLYWHEEL_ALLOWABLE_ERROR = 1.5;//ONLY ONE BEING USED
    public static final double FLYWHEEL_GEAR_REDUCTION = 1;
    public static final double FLYWHEEL_VELOCITY_CONSTANT = 0.028;
    public static final double FLYWHEEL_ACCELERATION_CONSTANT = 0.0030108;
    public static final double FLYWHEEL_SENSOR_POSITION_COEFFICIENT = (FLYWHEEL_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    public static final double FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT = FLYWHEEL_SENSOR_POSITION_COEFFICIENT * 10.0;
    public static final double FLYWHEEL_IDLE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2000);//OF THESE


    public static final int LEFT_SHOOTER_MOTOR_ID = 20;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 21;
    
    //TODO: Run SysID for shooter
    //TODO: Tune Shooter PID https://docs.wpilib.org/en/latest/docs/software/advanced-controls/introduction/tuning-flywheel.html

    public static final double kS = 6.624; // Add 0.25 V output to overcome static friction
    public static final double kV = .06325; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 2; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 8; // This will need to be tuned after feedforward
    public static final double kI = 0.2; // For flywheels, this should be 0
    public static final double kD = 0; // For flywheels, this should be 0

    /* Shooter Current Limiting */
    public static final int SHOOTER_CURRENT_LIMIT = 40;
    public static final int SHOOTER_SUPPLY_CURRENT_THRESHOLD = 40;
    public static final int SHOOTER_CURRENT_THRESHOLD = 65;
    public static final double SHOOTER_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;

    public static final boolean SHOOTER_STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 200;

    public static final double[][] SHOOTER_DISTANCE_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {1.95072, 95},
      {2.1336, 90},
      {2.286, 84},
      {2.4384, 46},
      {2.5908, 42.5},
      {2.7432, 36.9}
    };
    public static final MultiLinearInterpolator SHOOTER_DISTANCE_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(SHOOTER_DISTANCE_LOOKUP_TABLE);
}
