// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Currency;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.commands.FactoryCommands.State;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.util.PathOnTheFly;
import frc.robot.util.PathOnTheFly.AutoToPoint;

public class FactoryCommands extends SubsystemBase{
  private Arm arm;
  private Shooter shooter;
  private Intake intake;
  // private Candle candle;
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController xboxController;
  private ObjectDetection limelightObject;
  public FactoryCommands(Arm arm, Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain, ObjectDetection limelightCam, CommandXboxController xboxController){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    // this.candle = candle;
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
    this.limelightObject = limelightCam;
  }

  public Command shoot() {
    return Commands.either(ampShot(), speakerShoot(60,80), ()->(getState() == State.Amp));
  }

  public Command align() {
    return Commands.either(alignToAmp(), getInRange(), ()->(getState() == State.Amp));
  }

  public Command speakerShoot(double LSpeed, double RSpeed){
    return Commands.deadline(switchState(State.Speaker)
      .andThen(Commands.waitSeconds(.01)).andThen(Commands.waitUntil(() ->shooter.isShooterAtTargetSpeed()))
      .andThen(Commands.waitUntil(()->arm.isArmAtAngle())).andThen(intake.outtakePiece()).andThen(intake.stop())
      ,shooter.shootVelocity(LSpeed, RSpeed));
  }

  public Command ampShot(){
    return Commands.deadline(Commands.waitSeconds(.5)
      ,Commands.waitSeconds(0.003/*0.006*/).andThen(arm.setArmRotation(ArmState.AmpMove))
      ,intake.setVelocity(-18.6/*-19.25*/)).andThen(switchState(State.Speaker));
  }

  public Command intake(){
    return Commands.deadline(intake.intakePiece(),switchState(State.Intake))
      // .andThen(candle.pickUpLights())
      .andThen(Commands.runOnce(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .andThen(Commands.deadline(switchState(State.Speaker).andThen(Commands.waitSeconds(.5)).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)))
      .handleInterrupt(()->{
        xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        Command switchEnd = switchState(State.Speaker);
        switchEnd.initialize();
        switchEnd.schedule();
        intake.stop();
        // candle.idleLED();
      }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command intakeMainAuto(){
    return Commands.deadline(intake.intakePiece(),switchState(State.Intake))
      // .andThen(candle.pickUpLights())
      .andThen(Commands.runOnce(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .andThen(Commands.deadline(switchState(State.Speaker).andThen(Commands.waitSeconds(.3)).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)))
      .handleInterrupt(()->{
        xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        Command switchEnd = switchState(State.Speaker);
        switchEnd.initialize();
        switchEnd.schedule();
        intake.stop();
        // candle.idleLED();
      }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController thetaControllerAmp = new PIDController(6,0,.7);//(12.2,1.1,.4);
  public Command alignToAmp() {
    DoubleSupplier xAxis = () -> -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;

    thetaControllerAmp.reset();
    DoubleSupplier rotationalVelocity = () -> thetaControllerAmp.calculate(correctYaw((drivetrain.getPose().getRotation().getDegrees())%360,90), 90);
    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble())
      .withVelocityY(yAxis.getAsDouble())
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  
  private double correctYaw(double x, double setpoint){
    if (x>180+setpoint){
      x-=360;
    }else if(x<-180+setpoint){
      x+=360;
    }
    return x;
  }
  private final PIDController thetaControllerSpeaker = new PIDController(2,0,0.01);

  public Command alignToCorner() {
    DoubleSupplier xAxis = () -> -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();
    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromCorner().getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromCorner().getDegrees()-180,0);//-7;
      }else{
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromCorner().getDegrees()+180,0);//+7;
      }
    };
    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
      .withVelocityY(yAxis.getAsDouble())
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  private final PIDController thetaControllerPiece = new PIDController(3,0,0.2);
  private final PIDController distanceControllerPiece = new PIDController(1.7, 0, 0.2);
  public Command alignToPiece() {
    DoubleSupplier distanceSpeed = ()-> -distanceControllerPiece.calculate(limelightObject.getDistanceFromPieceVertical(), .5)+.5;
    DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getPose().getRotation().getDegrees()+limelightObject.getHorizontalRotationFromPiece().getDegrees()-90)*Math.PI/180;
    DoubleSupplier xAxis = () -> 
      (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> 
      (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();
    
    DoubleSupplier rotationalVelocity = () -> -thetaControllerPiece.calculate(-limelightObject.getHorizontalRotationFromPiece().getDegrees(),0);

    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble())
          .withVelocityY(yAxis.getAsDouble())
          .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }
  private final PIDController distanceControllerSpeaker = new PIDController(1.7, 0, 0.2);

  private final Pose2d speakerPose;
  {
     Pose2d pose = LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      pose = LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
    }
    speakerPose = pose;
  }

  public Command getInRange() {
    Pose2d alignPose = speakerPose;

    DoubleSupplier distanceSpeed = ()-> -distanceControllerSpeaker.calculate(drivetrain.getDistanceFromPoseMeters(alignPose), 2.395);
    DoubleSupplier redOrBlueSide = ()->{
      if (drivetrain.getPose().getX()>alignPose.getX()){
        return 90;
      }else{
        return -90;
      }
    };

    DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getPoseAngle(alignPose).getDegrees()+redOrBlueSide.getAsDouble())*Math.PI/180;
    DoubleSupplier xAxis = () -> 
      (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () ->
      (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();


    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromPose(alignPose).getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(alignPose).getDegrees()-180,0);//-7;
      }else{
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(alignPose).getDegrees()+180,0);//+7;
      }
    };

    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
          .withVelocityY(yAxis.getAsDouble())
          .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  public Command getToPieceCommand(){
    if (limelightObject.isPiecePresent()){
      return AutoToPoint.getToPoint(limelightObject.getPiecePose().get(),PathOnTheFly.getConfig(0));
    }
    return Commands.none();
  }
  public enum State{
    
    Amp,
    Speaker,
    Intake;
  }
  State state = State.Speaker;
  public Command switchState(State newState){
    switch (newState) {
        case Speaker:
            return arm.setArmDefault(ArmState.Speaker).alongWith(Commands.runOnce(()->state = State.Speaker));
        case Amp:
            return arm.setArmDefault(ArmState.Amp).alongWith(shooter.stopMotors()).alongWith(Commands.runOnce(()->state = State.Amp));
        case Intake:
            return arm.setArmDefault(ArmState.Intake).alongWith(Commands.runOnce(()->state = State.Intake));
    }
    return Commands.none();
  }

  public State getState(){
    return state;
  }

  public void periodic(){
    
  }
}