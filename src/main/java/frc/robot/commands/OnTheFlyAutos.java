// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.util.PathOnTheFly;
import frc.robot.util.PathOnTheFly.AutoToPoint;

/** Add your docs here. */
public class OnTheFlyAutos {
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    // private Candle candle;
    private CommandSwerveDrivetrain drivetrain;
    private ObjectDetection limelightObjectDetection;
    private CommandXboxController xboxController;
    private FactoryCommands groupCommands;
    public OnTheFlyAutos(Arm arm, Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain, ObjectDetection limelightCam, CommandXboxController xboxController, FactoryCommands groupCommands){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    // this.candle = candle;
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
    this.limelightObjectDetection = limelightCam;
    this.groupCommands = groupCommands;
  }
    public Command getVariable45() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180)));
    }else{
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    }
    
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("S-A 4.5 piece continuous")).andThen(Commands.either(
      AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("6 shoot"),
            new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)),
            3.0 
    ),AutoBuilder.followPath(PathPlannerPath.fromPathFile("6 to 7")).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("7 shoot"))),()->intake.isPiecePresent()));
  }

  public Command getOnTheFlyAuto() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.07,7.37), Rotation2d.fromDegrees(180)));
    }else{
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    }
    Command optionOne = new SequentialCommandGroup(Commands.deadline(groupCommands.intake(),groupCommands.alignToPiece()));
    Command optionTwo = new SequentialCommandGroup(AutoToPoint.getToPoint(new Pose2d(7.72,6.24,Rotation2d.fromDegrees(-37.12)),PathOnTheFly.getConfig(0),true),Commands.deadline(groupCommands.intake(),groupCommands.alignToPiece()));
    BooleanSupplier reason = ()->limelightObjectDetection.isPiecePresent() && CommandSwerveDrivetrain.poseWithinRange(limelightObjectDetection.getPiecePose().get(),new Pose2d(8.29,7.42,new Rotation2d()),.2);
    
    return Commands.sequence(AutoToPoint.getToPoint(new Pose2d(7.11,7.44,Rotation2d.fromDegrees(0)),PathOnTheFly.getConfig(0),true)
    ,Commands.either(optionOne, optionTwo, reason)
    ,AutoToPoint.getToPoint(new Pose2d(5.19,6.18,Rotation2d.fromDegrees(-172.21)),PathOnTheFly.getConfig(0),true)
    ,Commands.deadline(Commands.waitSeconds(3),groupCommands.getInRange())
    ,groupCommands.speakerShoot(60,80));
  }

  public Command onTheFlyAutoPiecePose() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.07,7.37), Rotation2d.fromDegrees(180)));
    }else{
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    }
    Command optionOne = new SequentialCommandGroup(Commands.deadline(groupCommands.intake(),groupCommands.getToPieceCommand()));
    Command optionTwo = new SequentialCommandGroup(AutoToPoint.getToPoint(new Pose2d(7.72,6.24,Rotation2d.fromDegrees(-37.12)),PathOnTheFly.getConfig(0),true),Commands.deadline(Commands.race(groupCommands.intake(), Commands.waitSeconds(2)),groupCommands.getToPieceCommand()));
    BooleanSupplier reason = ()->limelightObjectDetection.isPiecePresent() && CommandSwerveDrivetrain.poseWithinRange(limelightObjectDetection.getPiecePose().get(),new Pose2d(8.29,7.42,new Rotation2d()),.2);
    
    return Commands.sequence(AutoToPoint.getToPoint(new Pose2d(7.11,7.44,Rotation2d.fromDegrees(0)),PathOnTheFly.getConfig(0),true)
    ,Commands.either(optionOne, optionTwo, reason)
    ,AutoToPoint.getToPoint(new Pose2d(5.19,6.18,Rotation2d.fromDegrees(-172.21)),PathOnTheFly.getConfig(0),true)
    ,Commands.deadline(Commands.waitSeconds(3),groupCommands.getInRange())
    ,groupCommands.speakerShoot(60,80));
  }
}