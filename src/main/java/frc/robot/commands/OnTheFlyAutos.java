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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class OnTheFlyAutos {
  //Only need blue side because it flips
  private Pose2d[] piecePoses =  new Pose2d[]{
      new Pose2d(2.88,7.1,new Rotation2d())
    ,new Pose2d(2.88,5.6,new Rotation2d())
    ,new Pose2d(2.88,4.1,new Rotation2d())
    ,new Pose2d(8.27,7.4,new Rotation2d())
    ,new Pose2d(8.27,5.7,new Rotation2d())
    ,new Pose2d(8.27,4.1,new Rotation2d())
    ,new Pose2d(8.27,2.4,new Rotation2d())
    ,new Pose2d(8.27,0.8,new Rotation2d())};
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
  
  
}