// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.commands.PathOnTheFly.AutoToPoint;
import frc.robot.commands.PathOnTheFly.PathConfig;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.util.PoseEX;

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
    ,new Pose2d(8.27,0.8,new Rotation2d())
    ,new Pose2d(13.48,7.1,new Rotation2d())
    ,new Pose2d(13.48,5.6,new Rotation2d())
    ,new Pose2d(13.48,4.1,new Rotation2d())
  };
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
  
  /**
   * this will take in the indexes of every piece you want, numbered how Nityant wanted, if one is not present, it will go to the next one
   * give 0 if you want it to find a note on your side, if included should be the last command
   * @param initPose this pose will be flipped based on side, so make it on blue
   * @param places
   * @return
   */

  public Command getOnTheFlyAuto(Pose2d initPose, int... places){
    for (int i = 0; i < places.length; i++){
      if (places[i] > 8){
        places[i] -= 8;
      }
      places[i]--;
    }
    SequentialCommandGroup commandGroup = new SequentialCommandGroup(Commands.runOnce(()->shooter.setIdleSpeed(60,80)).andThen(shootAtSpeakerCommand()));
    for (int i : places){
      if (i == -1){
        commandGroup.addCommands(findIntakeAndShootNote());
      }else{
        commandGroup.addCommands(Commands.deadline(getToPiecePoseCommand(i).andThen(Commands.waitSeconds(.1)),groupCommands.intakeMainAuto())
          .andThen(Commands.either(shootAtSpeakerCommand(),Commands.none(),()->intake.isPiecePresent())));
      }
    }
    return Commands.defer(()->Commands.runOnce(()->{
        if (DriverStation.getAlliance().get() == Alliance.Red){
            drivetrain.seedFieldRelative(new Pose2d(16.54-initPose.getX(),initPose.getY(), Rotation2d.fromDegrees(180-initPose.getRotation().getDegrees())));
            return;
        }
        drivetrain.seedFieldRelative(initPose);
    }),Set.of()).andThen(commandGroup);
  }

  private Command getToPiecePoseCommand(int index){
    return Commands.either(groupCommands.getToPoseAndPoint(PoseEX.mirrorPose(piecePoses[index]), new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0))
    ,Commands.defer(()->AutoToPoint.getToPoint(PoseEX.getInbetweenPose2d(drivetrain.getPose(), piecePoses[index], .5)
    .transformBy(new Transform2d(0,0,PoseEX.getPoseAngle(drivetrain.getPose(),piecePoses[index])))
    //Rotation2d.fromDegrees(drivetrain.getPose().getRotation().getDegrees()+PoseEX.getPoseAngle(drivetrain.getPose(),piecePoses[index]).getDegrees()))
    ,new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0),true)
    , Set.of())
    ,()->DriverStation.getAlliance().get()==Alliance.Red
    );
  }

  private DeferredCommand shootAtSpeakerCommand(){
    Supplier<Pose2d> speakerPose = ()->{
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
        return LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
      }
      return LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
    };
    return new DeferredCommand(()->
      AutoToPoint.getToPoint(PoseEX.getInbetweenPose2d(drivetrain.getPose(),speakerPose.get(), 0)
      .transformBy(new Transform2d(0,0,PoseEX.getPoseAngle(drivetrain.getPose(),speakerPose.get())
      .plus(Rotation2d.fromDegrees(180)))),new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0))
      .until(()->PoseEX.getDistanceFromPoseMeters(speakerPose.get(),drivetrain.getPose())<4).andThen(Commands.deadline(Commands.waitSeconds(.5).andThen(groupCommands.speakerShoot(60,80)),groupCommands.getInRange())),Set.of(drivetrain,shooter,intake));
  }

  private Command findIntakeAndShootNote(){
    return groupCommands.autoFindNote().andThen(groupCommands.getToSpeakerCommand()).andThen(groupCommands.speakerShoot(60,80));
  }

}