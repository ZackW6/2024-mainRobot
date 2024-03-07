package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TargetWithShooterCommand extends Command {
    private final Shooter shooter;
    private final CommandSwerveDrivetrain swerve;
    
    static {
        // Old Limelight Tuning
        // SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
        // new Vector2(Math.toRadians(6.0),
        // Units.rotationsPerMinuteToRadiansPerSecond(1725)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
        // new Vector2(Math.toRadians(8.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(1825)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(2.0),
        // new Vector2(Math.toRadians(11.0),
        // Units.rotationsPerMinuteToRadiansPerSecond(1900)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
        // new Vector2(Math.toRadians(12.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(1950)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
        // new Vector2(Math.toRadians(14.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(2025)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
        // new Vector2(Math.toRadians(16.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(2100)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
        // new Vector2(Math.toRadians(19.0),
        // Units.rotationsPerMinuteToRadiansPerSecond(2200)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(4.5),
        // new Vector2(Math.toRadians(22.0),
        // Units.rotationsPerMinuteToRadiansPerSecond(2250)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(5.0),
        // new Vector2(Math.toRadians(24.0),
        // Units.rotationsPerMinuteToRadiansPerSecond(2325)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(5.5),
        // new Vector2(Math.toRadians(25.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(2425)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(6.0),
        // new Vector2(Math.toRadians(27.5),
        // Units.rotationsPerMinuteToRadiansPerSecond(2575)));

        // SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
        //         new Vector2(Math.toRadians(7.0), Units.rotationsPerMinuteToRadiansPerSecond(1725)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
        //         new Vector2(Math.toRadians(9.5), Units.rotationsPerMinuteToRadiansPerSecond(1725)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(2.0),
        //         new Vector2(Math.toRadians(11.5), Units.rotationsPerMinuteToRadiansPerSecond(1750)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
        //         new Vector2(Math.toRadians(13.5), Units.rotationsPerMinuteToRadiansPerSecond(1825)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
        //         new Vector2(Math.toRadians(16.0), Units.rotationsPerMinuteToRadiansPerSecond(1900)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
        //         new Vector2(Math.toRadians(18.5), Units.rotationsPerMinuteToRadiansPerSecond(2000)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
        //         new Vector2(Math.toRadians(21.5), Units.rotationsPerMinuteToRadiansPerSecond(2100)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(4.5),
        //         new Vector2(Math.toRadians(24.0), Units.rotationsPerMinuteToRadiansPerSecond(2175)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(5.0),
        //         new Vector2(Math.toRadians(25.5), Units.rotationsPerMinuteToRadiansPerSecond(2275)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(5.5),
        //         new Vector2(Math.toRadians(27.5), Units.rotationsPerMinuteToRadiansPerSecond(2400)));
        // SHOOTER_TUNING.put(new InterpolatingDouble(6.0),
        //         new Vector2(Math.toRadians(30.0), Units.rotationsPerMinuteToRadiansPerSecond(2475)));

    }
    public TargetWithShooterCommand(Shooter shooter, CommandSwerveDrivetrain swerve) {
        this.shooter = shooter;
        this.swerve = swerve;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = swerve.getDistanceFromSpeakerMeters();
        double power = distance;
        shooter.setTargetFlywheelSpeed(power);
    }
}