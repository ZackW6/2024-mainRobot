// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PoseEX {

    public static Rotation2d getPoseAngle(Pose2d mainPose, Pose2d comparingPose) {
        double deltaX = comparingPose.getX() - mainPose.getX();
        double deltaY = comparingPose.getY() - mainPose.getY();
        double angleRadians = ((Math.atan(deltaY/deltaX)));
        if (deltaX>0){
            return Rotation2d.fromRadians(angleRadians);
        }
        return Rotation2d.fromRadians(angleRadians+Math.PI);

    }

    public static Pose2d mirrorPose(Pose2d initPose){
        return new Pose2d(16.54-initPose.getX(),initPose.getY(),Rotation2d.fromDegrees(180-initPose.getRotation().getDegrees()));
    }
    
    public static double getDistanceFromPoseMeters(Pose2d mainPose, Pose2d comparingPose) {
        return Math.sqrt(Math.pow(comparingPose.getX()-mainPose.getX(),2)+Math.pow(comparingPose.getY()-mainPose.getY(),2));
    }
    
    public static Rotation2d getYawFromPose(Pose2d mainPose, Pose2d comparingPose) {
        double deltaX;
        double deltaY;
        deltaX = comparingPose.getX() - mainPose.getX();
        deltaY = comparingPose.getY() - mainPose.getY();

        double angleRadians = ((Math.atan(deltaY/deltaX)));

        // Convert the angle to Rotation2d
        Rotation2d rotation = Rotation2d.fromRadians(angleRadians - mainPose.getRotation().getRadians());
        if (mainPose.getX()>comparingPose.getX()){
            if (rotation.getDegrees()>0){
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - mainPose.getRotation().getDegrees()-180);
            }else{
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - mainPose.getRotation().getDegrees()+180);
            }
        }
        return rotation;
    }

    public static Rotation2d getPitchFromPose(Pose3d mainPose, Pose3d comparingPose) {
        double deltaDist;
        double deltaZ;
        deltaDist = Math.sqrt(Math.pow(comparingPose.getX() - mainPose.getX(),2)+Math.pow(comparingPose.getY() - mainPose.getY(),2));
        
        deltaZ = comparingPose.getZ() - mainPose.getZ();
        
        double angleZ = ((Math.atan(deltaZ/deltaDist)));
        return Rotation2d.fromRadians(angleZ - mainPose.getRotation().getY());
    }

    public static Pose2d getInbetweenPose2d(Pose2d mainPose, Pose2d comparingPose, double distFrom){

        double rise = comparingPose.getY()-mainPose.getY();
        double run = comparingPose.getX()-mainPose.getX();
        
        double amount = Math.sqrt((rise*rise)+(run*run));
        double newRise = rise*(distFrom/amount);
        double newRun = run*(distFrom/amount);

        double newX = comparingPose.getX() - newRun;
        double newY = comparingPose.getY() - newRise;
        Pose2d returnPose = new Pose2d(newX, newY, new Rotation2d());
        return returnPose;
    }
}
