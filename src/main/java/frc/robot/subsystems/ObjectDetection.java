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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.constants.LimelightConstants;
import frc.robot.simulationUtil.ObjectDetectionSim;
import frc.robot.util.PoseEX;

public class ObjectDetection extends SubsystemBase {
    
    private String limelightName;
    private Transform3d limelightTransform;
    private Supplier<Pose2d> robotPose = null;
    private ObjectDetectionSim objectDetectionSim;

    private Consumer<ObjectDetectionState> m_telemetryFunction = null;

    /** Creates a new ObjectDetection. */
    public ObjectDetection(String limelightName, Transform3d limelightTransform, Supplier<Pose2d> robotPose) {
        this.limelightName = limelightName;
        this.limelightTransform = limelightTransform;
        this.robotPose = robotPose;
        objectDetectionSim = new ObjectDetectionSim(limelightTransform, VecBuilder.fill(63.3,49.7), VecBuilder.fill(3,7), robotPose);
        objectDetectionSim.addObjectPose(new Pose3d(2.88,4.1,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(2.88,5.6,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(2.88,7.1,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(8.27,7.4,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(8.27,5.7,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(8.27,4.1,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(8.27,2.4,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(8.27,.8,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(13.661,4.1,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(13.661,5.6,0,new Rotation3d(0,0,0)));
        objectDetectionSim.addObjectPose(new Pose3d(13.661,7.1,0,new Rotation3d(0,0,0)));
    }

    public static class ObjectDetectionState{

        public Pose2d seenPose;

        public Pose2d intendedPose;

        public ObjectDetectionState(Pose2d seenPose, Pose2d intendedPose){
            this.seenPose = seenPose;
            this.intendedPose = intendedPose;
        }

    }

    public void registerTelemetry(Consumer<ObjectDetectionState> telemetryFunction) {
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
                m_telemetryFunction.accept(new ObjectDetectionState(getPiecePose(),objectDetectionSim.getClosestVisiblePiece().get().toPose2d()));
            } catch (Exception e) {
                m_telemetryFunction.accept(new ObjectDetectionState(new Pose2d(-1,-1, new Rotation2d()),new Pose2d(-1,-1, new Rotation2d())));
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
            double targetOffsetAngle_Vertical = getVerticalRotationFromPiece().getRadians();
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = limelightTransform.getRotation().getY();

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightMeters = limelightTransform.getZ();

            // distance from the target to the floor
            double goalHeightMeters = 0;

            double angleToGoalRadians = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

            return  (limelightLensHeightMeters-goalHeightMeters) / -(Math.sin(angleToGoalRadians));
        }
        return 0;
    }

    public double getDistanceFromPieceHorizontal(){
        if (isPiecePresent()){
            double targetOffsetAngle_Horizontal = getHorizontalRotationFromPiece().getRadians();

            // what is the yaw of your limelight
            double limelightMountAngleRadians = limelightTransform.getRotation().getZ(); 

            double angleToGoalRadians = limelightMountAngleRadians + targetOffsetAngle_Horizontal;
            return getDistanceFromPieceVertical()*Math.tan(angleToGoalRadians);
        }
        return 0;
    }
    public Pose2d getPiecePose(){
        if (isPiecePresent()){
            Pose3d initialPose = new Pose3d(robotPose.get()).transformBy(limelightTransform);
            double initialPitch = initialPose.getRotation().getY(); // Initial pose's pitch in radians
            double pitch = Math.toRadians(getVerticalRotationFromPiece().getDegrees()); // Target pitch in radians
            double yaw = Math.toRadians(getHorizontalRotationFromPiece().getDegrees()); // Target yaw in radians
        
            // Calculate total pitch and yaw
            double totalPitch = initialPitch + pitch;
            double totalYaw = initialPose.getRotation().getZ() + yaw;
        
            // Calculate direction vector components
            double dx = Math.cos(totalPitch) * Math.cos(totalYaw);
            double dy = Math.cos(totalPitch) * Math.sin(totalYaw);
            double dz = Math.sin(totalPitch);

            // Initial position
            double x0 = initialPose.getX();
            double y0 = initialPose.getY();
            double z0 = initialPose.getZ();
        
            // Calculate distance to the floor
            double t = z0 / -dz;
        
            // Calculate new position
            double newX = x0 + t * dx;
            double newY = y0 + t * dy;
            double newZ = 0; // Object is resting on the floor
        
            // Create new orientation (pitch and yaw)
            Rotation3d newRotation = new Rotation3d(totalPitch, 0, totalYaw);
        
            // Create and return new pose
            return new Pose3d(newX, newY, newZ, newRotation).toPose2d();
            // Pose2d pose = robotPose.get();
            // double x = getDistanceFromPieceVertical();
            // double y = getDistanceFromPieceHorizontal();
            // double xn = x*Math.cos(pose.getRotation().getRadians())- y*Math.sin(pose.getRotation().getRadians());
            // double yn = x*Math.sin(pose.getRotation().getRadians())+ y*Math.cos(pose.getRotation().getRadians());
            // return new Pose2d(xn+pose.getX(),yn+pose.getY(),Rotation2d.fromDegrees(PoseEX.getYawFromPose(pose, new Pose2d(xn+pose.getX(),yn+pose.getY(), new Rotation2d())).getDegrees()+robotPose.get().getRotation().getDegrees()));
        }
        return new Pose2d(-1,-1,new Rotation2d());
    }

    public ObjectDetectionSim getSim(){
        return objectDetectionSim;
    }
}
