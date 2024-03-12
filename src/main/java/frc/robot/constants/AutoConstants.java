package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
    public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIGS = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(11, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.323, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double K_PX_CONTROLLER = 4;
    public static final double K_PY_CONTROLLER = 4;
    public static final double K_P_THETA_CONTROLLER = 4.5;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
}
