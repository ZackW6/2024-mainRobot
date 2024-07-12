// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
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
import frc.robot.commands.PathOnTheFly.AutoToPoint;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ChoreoEX;
import frc.robot.util.PoseEX;

/** Add your docs here. */
public class ConditionalAutos {
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    // private Candle candle;
    private CommandSwerveDrivetrain drivetrain;
    private ObjectDetection limelightObjectDetection;
    private CommandXboxController xboxController;
    private FactoryCommands groupCommands;
    public ConditionalAutos(Arm arm, Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain, ObjectDetection limelightCam, CommandXboxController xboxController, FactoryCommands groupCommands){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    // this.candle = candle;
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
    this.limelightObjectDetection = limelightCam;
    this.groupCommands = groupCommands;
  }
  public Command getConditionalAuto() {
    // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180)));
    // }else{
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0)));
    // }
    BooleanSupplier condition = ()->intake.isPiecePresent();
    Command continueAt8 = 
      Commands.either(ChoreoEX.getChoreoPath("shoot8M")
      ,groupCommands.autoFindNote().andThen(Commands.deadline(Commands.waitSeconds(2000).until(()->(
        PoseEX.getDistanceFromPoseMeters(drivetrain.getPose(), LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d())<3.7 || 
        PoseEX.getDistanceFromPoseMeters(drivetrain.getPose(), LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d())<3.7))
        .andThen(groupCommands.speakerShoot(60,80)),groupCommands.getToSpeakerCommand())), condition);
    Command continueAt7 = 
      Commands.either(ChoreoEX.getChoreoGroupPath(false, new String[]{"shoot7M","intake8"})
      ,ChoreoEX.getChoreoPath("ifNo7Then8"), condition).andThen(continueAt8);
    Command continueAt6 = 
      Commands.either(ChoreoEX.getChoreoGroupPath(false, new String[]{"shoot6M","intake7"})
      ,ChoreoEX.getChoreoPath("ifNo6Then7"), condition).andThen(continueAt7);
    Command continueAt5 = 
      Commands.either(ChoreoEX.getChoreoGroupPath(false, new String[]{"shoot5M","intake6"})
      ,ChoreoEX.getChoreoPath("ifNo5Then6"), condition).andThen(continueAt6);
    return ChoreoEX.getChoreoGroupPath(true, new String[]{"shootPreAmp","intake4"})
    .andThen(Commands.either(ChoreoEX.getChoreoGroupPath(false, new String[]{"shoot4M","intake5"})
    , ChoreoEX.getChoreoPath(false, "ifNo4Then5")
    , condition)).andThen(continueAt5);
    // return Commands.defer(()->Commands.either(
    //   Commands.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180))))
    //   , Commands.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0))))
    //   , ()->(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red))),Set.of(drivetrain))
    //   .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("S-A 4.5 piece continuous"))).andThen(Commands.either(
    //   AutoBuilder.pathfindThenFollowPath(
    //         PathPlannerPath.fromPathFile("6 shoot"),
    //         new PathConstraints(
    //         3.0, 4.0,
    //         Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //         3.0 
    // ),AutoBuilder.followPath(PathPlannerPath.fromPathFile("6 to 7")).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("7 shoot"))),()->intake.isPiecePresent()));
  }

  // public Command onTheFlyAutoPiecePose() {
  //   Command optionOne = new SequentialCommandGroup(Commands.deadline(groupCommands.intake(),groupCommands.getToPieceCommand()));
  //   Command optionTwo = new SequentialCommandGroup(AutoToPoint.getToPoint(new Pose2d(7.72,6.24,Rotation2d.fromDegrees(-37.12)),PathOnTheFly.getConfig(0),true),Commands.deadline(Commands.race(groupCommands.intake(), Commands.waitSeconds(2)),groupCommands.getToPieceCommand()));
  //   BooleanSupplier reason = ()->limelightObjectDetection.isPiecePresent() && CommandSwerveDrivetrain.poseWithinRange(limelightObjectDetection.getPiecePose(),new Pose2d(8.29,7.42,new Rotation2d()),1);
    
  //   return Commands.defer(()->Commands.either(
  //     Commands.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.09,7.37), Rotation2d.fromDegrees(180))))
  //     , Commands.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.49,7.37), Rotation2d.fromDegrees(0))))
  //     , ()->(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red))),Set.of(drivetrain))
  //     .andThen(Commands.sequence(AutoToPoint.getToPoint(new Pose2d(7.11,7.44,Rotation2d.fromDegrees(0)),PathOnTheFly.getConfig(0),true)
  //   ,Commands.either(optionOne, optionTwo, reason)
  //   ,AutoToPoint.getToPoint(new Pose2d(5.19,6.18,Rotation2d.fromDegrees(-172.21)),PathOnTheFly.getConfig(0),true)
  //   ,Commands.deadline(Commands.waitSeconds(3),groupCommands.getInRange())
  //   ,groupCommands.speakerShoot(60,80)));
  // }
}