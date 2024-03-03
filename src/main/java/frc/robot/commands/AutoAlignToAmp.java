// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.CommandSwerveDrivetrain;
// import frc.robot.Robot;

// public class AutoAlignToAmp extends Command {
//       private final CommandSwerveDrivetrain drivetrain;

//     private final PIDController controller = new PIDController(5,0,.3);

//     private final DoubleSupplier xAxis;
//     private final DoubleSupplier yAxis;
//   /** Creates a new AutoAlignToAmp. */
//   public AutoAlignToAmp(CommandSwerveDrivetrain drivetrain, DoubleSupplier xAxis,
//   DoubleSupplier yAxis) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drivetrain = drivetrain;
//     this.xAxis = xAxis;
//     this.yAxis = yAxis;

//     controller.enableContinuousInput(-Math.PI, Math.PI);
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     controller.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     Rotation2d currentAngle = drivetrain.getYawRadians();

//     controller.setSetpoint(Math.PI);

//     double rotationalVelocity = controller.calculate(currentAngle.getRadians());
//     // 
//     // drivetrain.applyRequest(() -> drivetrain.withVelocityX(-xAxis.getAsDouble()) // Drive forward with
//     // // negative Y (forward)
//     // .withVelocityY(-yAxis.getAsDouble()) // Drive left with negative X (left)
//     // .withRotationalRate(rotationalVelocity) // Drive counterclockwise with negative X (left)
//     // ));
//     // drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(),
//     //         -rotationalVelocity, currentAngle));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
