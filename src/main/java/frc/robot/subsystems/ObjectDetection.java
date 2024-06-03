// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.constants.LimelightConstants;
import frc.robot.simulationUtil.ObjectDetectionSim;

public class ObjectDetection extends SubsystemBase {
    private String limelightName;
    private Transform3d limelightTransform;
    private Supplier<Pose2d> robotPose = null;
    private ObjectDetectionSim objectDetectionSim;

    private Consumer<Pose2d> m_telemetryFunction = null;

    /** Creates a new ObjectDetection. */
    public ObjectDetection(String limelightName, Transform3d limelightTransform, Supplier<Pose2d> robotPose) {
        this.limelightName = limelightName;
        this.limelightTransform = limelightTransform; 
        this.robotPose = robotPose;
        objectDetectionSim = new ObjectDetectionSim(limelightTransform, VecBuilder.fill(63.3,49.7), VecBuilder.fill(3,7), robotPose);
        objectDetectionSim.addObjectPose(new Pose3d(5,5,1,new Rotation3d(0,0,0)));
    }

    public void registerTelemetry(Consumer<Pose2d> telemetryFunction) {
        m_telemetryFunction = telemetryFunction;
    }

    @Override
    public void periodic() {
        // System.out.println(getDistanceFromPieceVertical());
        // if (isPiecePresent()){
        //     System.out.println(getPiecePose().get());
        // }
        // System.out.println(Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-object")));
        if (m_telemetryFunction!=null){
            try {
                m_telemetryFunction.accept(getPiecePose().get());
            } catch (Exception e) {
                m_telemetryFunction.accept(new Pose2d(-1,-1, new Rotation2d()));
            }
            
        }
    }

    public Rotation2d getHorizontalRotationFromPiece(){
        if (Robot.isSimulation()){
            return objectDetectionSim.getHorizontalRotationFromTarget();
        }
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        NetworkTableEntry tx = table.getEntry("tx");
        return Rotation2d.fromDegrees(tx.getDouble(0.0));
    }
    
    public Rotation2d getVerticalRotationFromPiece(){
        if (Robot.isSimulation()){
            return objectDetectionSim.getVerticalRotationFromTarget();
        }
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        NetworkTableEntry ty = table.getEntry("ty");
        return Rotation2d.fromDegrees(ty.getDouble(0.0));
    }

    public boolean isPiecePresent(){
        if (Robot.isSimulation()){
            return objectDetectionSim.isPiecePresent();
        }
        if (LimelightHelpers.getTA(limelightName) != 0){
            return true;
        }
        return false;
    }

    public double getDistanceFromPieceVertical(){
        if (isPiecePresent()){
            double targetOffsetAngle_Vertical = getVerticalRotationFromPiece().getDegrees();
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = Units.radiansToDegrees(limelightTransform.getRotation().getY()); 

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = Units.metersToInches(limelightTransform.getZ()); 

            // distance from the target to the floor
            double goalHeightInches = 1; 

            double angleToGoalDegrees = -limelightMountAngleDegrees + targetOffsetAngle_Vertical;

            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            return  Units.inchesToMeters((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians))+limelightTransform.getY();
        }
        return 0;
    }
    public double getDistanceFromPieceHorizontal(){
        if (isPiecePresent()){
            double targetOffsetAngle_Horizontal = getHorizontalRotationFromPiece().getDegrees();
            
            // what is the yaw of your limelight
            double limelightMountAngleDegrees = Units.radiansToDegrees(limelightTransform.getRotation().getZ()); 

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Horizontal;

            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            return getDistanceFromPieceVertical()*Math.tan(angleToGoalRadians);//+LimelightConstants.AMP_CAM_TRANSFORM.getX();
        }
        return 0;
    }
    public Optional<Pose2d> getPiecePose(){
        if (isPiecePresent() && robotPose != null){
            Pose2d pose = robotPose.get();
            double x = getDistanceFromPieceVertical();
            double y = getDistanceFromPieceHorizontal();
            double xn = x*Math.cos(pose.getRotation().getRadians())- y*Math.sin(pose.getRotation().getRadians());
            double yn = x*Math.sin(pose.getRotation().getRadians())+ y*Math.cos(pose.getRotation().getRadians());
            return Optional.of(new Pose2d(xn+pose.getX(),yn+pose.getY(),new Rotation2d()));
        }
        return null;
    }
}
