package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants{
        public static final int ARM_MOTOR_ID = 10;
        public static final int CAN_CODER_ID = 10;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 190;//300;
        public static final double kI = .2;//.1;
        public static final double kD = 50;//55;
        public static final double kG = 8.574609375;//8.4749;

        // TODO: Make the arm positive when it goes up.
        public static final double CRUISE_VELOCITY = 400;
        public static final double MAX_ACCELERATION = 1000;
		public static final double JERK = 5000;

        
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.592);//.592);//.063);//.314);//
        

        /* Arm Current Limiting */ //TODO: Change these values
        public static final int ARM_CURRENT_LIMIT = 40;
        public static final int ARM_SUPPLY_CURRENT_THRESHOLD = 65;
        public static final int ARM_CURRENT_THRESHOLD = 70;
        public static final double ARM_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ARM_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ARM_STATOR_CURRENT_LIMIT_ENABLE = true;
		public static final double ARM_STATOR_CURRENT_LIMIT = 200;
        
    }
