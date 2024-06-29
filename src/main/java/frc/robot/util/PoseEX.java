// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PoseEX {

    public static Rotation2d getPoseAngle(Pose2d mainPose, Pose2d comparingPose) {
        double deltaX = comparingPose.getX() - mainPose.getX();
        double deltaY = comparingPose.getY() - mainPose.getY();
        double angleRadians = ((Math.atan(deltaY/deltaX)));
        return Rotation2d.fromRadians(angleRadians);
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
        return Rotation2d.fromRadians(angleZ + mainPose.getRotation().getY());
    }

    public static Pose2d getInbetweenPose2d(Pose2d mainPose, Pose2d comparingPose, double distFrom){
        double rise = mainPose.getY()-comparingPose.getY();
        double run = mainPose.getX()-comparingPose.getX();
        if (rise < 0){
            if (run < 0){
                run = -run;
                rise = -rise;
            }
        }else{
            if (run < 0){
                rise = -rise;
                run = -run;
            }
        }
        if (run == 0){
            run = .0001;
        }
        
        double amount = Math.sqrt((rise*rise)+(run*run));
        double newRise = rise*(distFrom/amount);
        double newRun = run*(distFrom/amount);
        Pose2d returnPose = mainPose.transformBy(new Transform2d(newRun,newRise,Rotation2d.fromDegrees(-mainPose.getRotation().getDegrees())));
        return returnPose;
    }
}
