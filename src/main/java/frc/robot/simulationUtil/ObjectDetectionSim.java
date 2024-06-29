// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationUtil;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.util.PoseEX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Simple sim to test object detection related ideas*/
public class ObjectDetectionSim {
    private final Transform3d transform3d;
    private final Vector<N2> fieldOfView;
    private final ArrayList<Pose3d> possibleObjects;
    private final Supplier<Pose2d> robotPose;
    /**
     * @param transform from center of robot
     * @param FOV horizontal and vertical FOV in degrees
     */
    public ObjectDetectionSim(Transform3d transform, Vector<N2> FOV, Vector<N2> latencyRange, Supplier<Pose2d> robotPose){
        this.transform3d = transform;
        this.fieldOfView = FOV;
        this.possibleObjects = new ArrayList<Pose3d>();
        this.robotPose = robotPose;
    }

    public void addObjectPose(Pose3d pose){
        possibleObjects.add(pose);
    }

    public void removeObjectPose(Pose3d pose){
        possibleObjects.remove(pose);
    }

    public ArrayList<Pose3d> getObjectPoses(){
        return possibleObjects;
    }

    public boolean isPiecePresent(){
        if (getClosestVisiblePiece().isEmpty()){
            return false;
        }
        return true;
    }
    
    public Optional<Pose3d> getClosestVisiblePiece(){
        ArrayList<Pose3d> objects = new ArrayList<Pose3d>();
        Pose3d cameraPose = new Pose3d(robotPose.get()).transformBy(transform3d);
        for (Pose3d pose : possibleObjects){
            Rotation2d yawFromPiece = PoseEX.getYawFromPose(cameraPose.toPose2d(), pose.toPose2d());
            Rotation2d pitchFromPiece = PoseEX.getPitchFromPose(cameraPose, pose);
            // System.out.println(pitchFromPiece.getDegrees()+" PICTH");
            double dist = Math.sqrt(Math.pow(pose.toPose2d().getX()-cameraPose.getX(),2)+Math.pow(pose.toPose2d().getY()-cameraPose.getY(),2));
            if (pitchFromPiece.getRadians()<Units.degreesToRadians(fieldOfView.get(1)/2)
                    && pitchFromPiece.getRadians()>-Units.degreesToRadians(fieldOfView.get(1)/2)
                    && yawFromPiece.getRadians()<Units.degreesToRadians(fieldOfView.get(0)/2)
                    && yawFromPiece.getRadians()>-Units.degreesToRadians(fieldOfView.get(0)/2)
                    && dist<6){
                objects.add(pose);
            }
        }
        if (objects.size()!=0){
            double minDist = Integer.MAX_VALUE;
            Pose3d minObject = objects.get(0);
            for (Pose3d pose : objects){
                double curDist = Math.sqrt(Math.pow(cameraPose.getX()-pose.getX(),2)+Math.pow(cameraPose.getX()-pose.getY(),2)+Math.pow(cameraPose.getX()-pose.getZ(),2));
                if (curDist<minDist){
                    minDist = curDist;
                    minObject = pose;
                }
            }
            return Optional.of(minObject);
        }
        return Optional.empty();
        
    }

    public Rotation2d getHorizontalRotationFromTarget(){
        Pose3d cameraPose = new Pose3d(robotPose.get()).transformBy(transform3d);
        try {
            return PoseEX.getYawFromPose(cameraPose.toPose2d(),getClosestVisiblePiece().get().toPose2d());
        } catch (Exception e) {
            return Rotation2d.fromDegrees(0);
        }
        
    }
    
    public Rotation2d getVerticalRotationFromTarget(){
        Pose3d cameraPose = new Pose3d(robotPose.get()).transformBy(transform3d);
        try {
            return PoseEX.getPitchFromPose(cameraPose,getClosestVisiblePiece().get());
        } catch (Exception e) {
            return Rotation2d.fromDegrees(0);
        }
    }
}
