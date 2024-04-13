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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
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
  public FactoryCommands(Arm arm, Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain, CommandXboxController xboxController){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    // this.candle = candle;
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
  }

  public Command shoot() {
    return Commands.either(ampShot(), loadAndShoot(), ()->arm.isArmInAmpState()) ;
  }
  public Command loadAndShoot(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed()))
      ,Commands.run(() -> {
        // double shooterSpeed = ShooterConstants.SHOOTER_DISTANCE_LINEAR_INTERPOLATOR.getLookupValue(drivetrain.getDistanceFromSpeakerMeters()+.2)[0];
        shooter.setTargetFlywheelSpeed(78, 78);
      }), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }

  public Command loadAndShootAuto(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed()))
      ,Commands.run(() -> shooter.setTargetFlywheelSpeed(75,75)), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }

  public Command loadAndShootAutoSecondary(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed() && shooter.isRightFlywheelAtTargetSpeed()))
      ,Commands.run(() -> shooter.setTargetFlywheelSpeed(50,35)), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }

  public Command loadAndShootAutoTertiary(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed() && shooter.isRightFlywheelAtTargetSpeed()))
      ,Commands.run(() -> shooter.setTargetFlywheelSpeed(55,37)), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }
  public Command loadAndShootOperator(){
    return Commands.deadline(Commands.waitSeconds(.01).andThen(Commands.waitUntil(() ->shooter.isLeftFlywheelAtTargetSpeed() && shooter.isRightFlywheelAtTargetSpeed()))
      ,Commands.run(() -> shooter.setTargetFlywheelSpeed(60,60)), Commands.runOnce(()->shooter.disableDefault()))
      .andThen(Commands.waitUntil(()->arm.isArmInSpeakerState())).andThen(intake.outtakePiece())
      .andThen(resetAll());
  }

  public Command ampShot(){
    return Commands.deadline(Commands.waitSeconds(.5)
      ,Commands.waitSeconds(0.003/*0.006*/).andThen(Commands.runOnce(()->arm.setCurrentArmState(ArmState.AmpMove)))
      ,intake.setVelocity(-18.8/*-19.25*/)).andThen(switchModes());
  }
  @Deprecated
  public Command intakeShoot(){
    return Commands.deadline(Commands.waitSeconds(2)
      ,Commands.runOnce(()->arm.setCurrentArmState(ArmState.IntakeShoot))
      ,Commands.deadline(Commands.waitSeconds(.1),intake.setVelocity(10)).andThen(intake.setVelocity(-100000/*-19.25*/))).andThen(Commands.runOnce(()->arm.setCurrentArmState(ArmState.Amp))).andThen(switchModes());
  }

  public Command intake(){
    return Commands.deadline(intake.intakePiece(),Commands.runOnce(()->arm.setCurrentArmState(ArmState.Intake)),Commands.runOnce(()->shooter.enableDefault()))
      // .andThen(candle.pickUpLights())
      .andThen(Commands.runOnce(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .andThen(Commands.deadline(Commands.waitSeconds(.5).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(25)
      ,Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState()))))
      .andThen(intake.stop()).finallyDo(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 0));
      // .andThen(candle.idleLED());
  }

  public Command intakeMainAuto(){
    return Commands.race(Commands.waitSeconds(2.5)
      ,Commands.deadline(intake.intakePiece(),Commands.runOnce(()->arm.setCurrentArmState(ArmState.Intake)),Commands.runOnce(()->shooter.enableDefault()))
      )// .andThen(candle.pickUpLights()))
      .andThen(Commands.deadline(Commands.waitSeconds(.5).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)
      ,Commands.runOnce(()->arm.setCurrentArmState(arm.lastMainState()))))
      .andThen(intake.stop());
      // .andThen(candle.idleLED());
  }

  public Command resetAll(){
    return Commands.runOnce(()->shooter.enableDefault())
      .andThen(intake.stop())
      .andThen(Commands.runOnce(()->arm.setCurrentArmState(ArmState.Speaker)));
  }



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController thetaControllerAmp = new PIDController(12.2,0,.5);//(12.2,1.1,.4);
  public Command alignToAmp() {
    DoubleSupplier xAxis = () -> -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerAmp.reset();
    DoubleSupplier rotationalVelocity = () -> thetaControllerAmp.calculate(correctYaw((drivetrain.getPose().getRotation().getDegrees())%360,90), 90);
    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) //was -xAxis, but in sim is this
      .withVelocityY(yAxis.getAsDouble())//was -yAxis, but in sim is this
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())))
      .alongWith(setCandleAmpLights());
  }
  private Command setCandleAmpLights(){
    DoubleSupplier teamID = ()->{
      if (DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
          return 5;
        }
      }
      return 6;
    };
    return Commands.run(()->{
      if (drivetrain.getPose().getX()<LimelightConstants.K_TAG_LAYOUT.getTagPose((int)teamID.getAsDouble()).get().getX()+.127
    && drivetrain.getPose().getX()>LimelightConstants.K_TAG_LAYOUT.getTagPose((int)teamID.getAsDouble()).get().getX()-.127){
        // candle.ampAlignLights().schedule();
      }else{
        // candle.idleLED().schedule();
      }
    });
  }
  private double correctYaw(double x, double setpoint){
    if (x>180+setpoint){
      x-=360;
    }else if(x<-180+setpoint){
      x+=360;
    }
    return x;
  }
  private final PIDController thetaControllerSpeaker = new PIDController(3.5,0,0.01);
  private final PIDController distanceController = new PIDController(1.9, 0, 0.2);
  
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
  private final PIDController thetaControllerPiece = new PIDController(1.9,0,0.2);
  public Command alignToPiece() {
    DoubleSupplier distanceSpeed = ()-> -distanceController.calculate(drivetrain.getDistanceFromPieceVertical(), 0);
    DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getPose().getRotation().getDegrees()-drivetrain.getRotationFromPiece(LimelightConstants.AMP_CAM).getDegrees()-90)*Math.PI/180;
    DoubleSupplier xAxis = () -> 
      (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> 
      (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();
    
    DoubleSupplier rotationalVelocity = () -> -thetaControllerPiece.calculate(drivetrain.getRotationFromPiece(LimelightConstants.AMP_CAM).getDegrees(),0);

    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
          .withVelocityY(yAxis.getAsDouble())
          .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }
  // public Command alignToPiece() {
  //   thetaControllerPiece.reset();
  //   DoubleSupplier rotationalVelocity = () -> -thetaControllerPiece.calculate(drivetrain.getAngleFromPose(new Pose2d(0,0, new Rotation2d())) /*getRotationFromPiece(LimelightConstants.AMP_CAM)*/.getDegrees(),0);
  //   DoubleSupplier distanceSpeed = ()-> distanceController.calculate(drivetrain.getDistanceFromPoseMeters(new Pose2d(0,0, new Rotation2d())), 0);
  //   DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getAngleFromPose(new Pose2d(0,0, new Rotation2d()))/*drivetrain.getRotationFromPiece(LimelightConstants.AMP_CAM)*/.getDegrees())*Math.PI/180;
  //   DoubleSupplier xAxis = () -> 
  //     (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
  //     /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
  //     -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
  //   DoubleSupplier yAxis = () -> 
  //     (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
  //     /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
  //     -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
  //   return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
  //     .withVelocityY(yAxis.getAsDouble())
  //     .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  // }

  public Command getInRange() {
    // DoubleSupplier xAxis = () -> -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    // DoubleSupplier yAxis = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    Supplier<Pose2d> teamID = ()->{
      if (DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
          return LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
        }
      }
      return LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
    };
    DoubleSupplier distanceSpeed = ()-> -distanceController.calculate(drivetrain.getDistanceFromPoseMeters(teamID.get()), 2.32);
    DoubleSupplier redOrBlueSide = ()->{
      if (drivetrain.getPose().getX()>teamID.get().getX()/*!alliance.isPresent() || alliance.get().equals(Alliance.Blue)*/){
        return 90;
      }else{
        return -90;
      }
    };
    DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getPoseAngle(teamID.get()).getDegrees()+redOrBlueSide.getAsDouble())*Math.PI/180;
    DoubleSupplier xAxis = () -> 
      (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> 
      (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
      /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
      -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    // DoubleSupplier rotation = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();
    
    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromPose(teamID.get()).getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(teamID.get()).getDegrees()-180,0);//-7;
      }else{
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(teamID.get()).getDegrees()+180,0);//+7;
      }
    };

    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
          .withVelocityY(yAxis.getAsDouble())
          .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  public Command switchModes(){
    return Commands.runOnce(() -> {
      if (arm.getCurrentArmState()!=ArmState.Intake){
        ArmState prevState = arm.getCurrentArmState();
        ArmState setState;
        if (prevState == ArmState.Speaker){
          shooter.disableFlywheel();
          setState=ArmState.Amp;
        }else{
          shooter.enableFlywheel();
          setState=ArmState.Speaker;
        }
        arm.setCurrentArmState(setState);
      }
      });
  }
  private double startingGyro;
  private double startingPosition; 
  @Deprecated
  public Command wheelRadiusCommand(){
    return Commands.deadline(Commands.runOnce(()->{
      startingGyro = drivetrain.getPigeon2().getAngle();
      startingPosition = drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble()/6.75;
    })
    .andThen(Commands.waitSeconds(30)
    .andThen(Commands.runOnce(()->{
      System.out.println(Math.abs(Math.abs(startingGyro)-Math.abs(drivetrain.getPigeon2().getAngle()))+"GYRO CHANGE");
      System.out.println(Math.abs(Math.abs(startingPosition)-Math.abs(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble()/6.75))+"POS CHANGE");
      double wheelRadius = Units.degreesToRadians(Math.abs(Math.abs(startingGyro)-Math.abs(drivetrain.getPigeon2().getAngle())))*13.258252/Units.rotationsToRadians(Math.abs(Math.abs(startingPosition)-Math.abs(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble()/6.75)));
      System.out.println(wheelRadius);
    }))),
    drivetrain.applyRequest(() -> drive.withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(Units.degreesToRadians(120))));
  }
  public Command getToPieceCommand(){
    if (drivetrain.isPiecePresent()){
      return AutoToPoint.getToPoint(drivetrain.getPiecePose().get().getX()
      ,drivetrain.getPiecePose().get().getY()
      ,drivetrain.getPose().getRotation().getDegrees()+drivetrain.getRotationFromPiece(LimelightConstants.AMP_CAM).getDegrees()
      ,PathOnTheFly.getConfig(0));
    }else{
      return Commands.none();
    }
  }
}