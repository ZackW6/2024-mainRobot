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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Simple sim to test object detection related ideas*/
public class ObjectDetectionSim {
    private final Transform3d transform3d;
    private final Vector<N2> fieldOfView;
    private final ArrayList<Pose3d> possibleObjects;
    /**
     * @param transform from center of robot
     * @param FOV horizontal and vertical FOV in degrees
     */
    public ObjectDetectionSim(Transform3d transform, Vector<N2> FOV, Vector<N2> latencyRange, Supplier<Pose2d> robotPose){
        this.transform3d = transform;
        this.fieldOfView = FOV;
        this.possibleObjects = new ArrayList<Pose3d>();
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
        if (getClosestVisiblePiece()!=null){
            return true;
        }
        return false;
    }

    private Optional<Pose3d> getClosestVisiblePiece(){
        ArrayList<Pose3d> objects;
        for (Pose3d pose : possibleObjects){
            if (pose.getX()>0){
                
            }
        }
        return null;
    }

    public Rotation2d getHorizontalRotationFromTarget(){
        return Rotation2d.fromDegrees(0);
    }
    
    public Rotation2d getVerticalRotationFromTarget(){
        return Rotation2d.fromDegrees(0);
    }
}
