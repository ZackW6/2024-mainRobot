// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Helper class for making on the fly paths
 */
public class PathOnTheFly {
        private static PathConfig[] configs = new PathConfig[4];
        public static class AutoToPath{
                public static Command getToPath(String name){
                        PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(3,4,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),0,0);
                        return getToPath(name, pathConfig);
                }
                public static Command getToPath(String name, PathConfig config){
                        PathPlannerPath path = PathPlannerPath.fromPathFile(name);


                        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
                        PathConstraints constraints = new PathConstraints(
                                config.maxVelocity, config.maxAcceleration,
                                Units.degreesToRadians(config.maxAngularVelocity.getDegrees()), Units.degreesToRadians(config.maxAngularAcceleration.getDegrees()));


                        // Since AutoBuilder is configured, we can use it to build pathfinding commands
                        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                path,
                                constraints,
                                config.rotationDelay // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                        );
                        return pathfindingCommand;
                }
        }
        public static class AutoToPoint{
                public static Command getToPoint(Pose2d pose, PathConfig config, boolean flip){
                        // Create the constraints to use while pathfinding
                        PathConstraints constraints = new PathConstraints(
                                config.maxVelocity, config.maxAcceleration,
                                Units.degreesToRadians(config.maxAngularVelocity.getDegrees()), Units.degreesToRadians(config.maxAngularAcceleration.getDegrees()));
                        // Since AutoBuilder is configured, we can use it to build pathfinding commands
                        boolean flipped = false;
                        if (flip){
                                flipped = true;
                        }
                        if (flipped){
                                return AutoBuilder.pathfindToPoseFlipped(
                                        pose,
                                        constraints,
                                        config.endVelocity, // Goal end velocity in meters/sec
                                        config.rotationDelay // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                );
                        }else{
                                return AutoBuilder.pathfindToPose(
                                        pose,
                                        constraints,
                                        config.endVelocity, // Goal end velocity in meters/sec
                                        config.rotationDelay // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                );
                        }              
                }
                public static Command getToPoint(Pose2d pose){
                        PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(3,4,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),0,0);
                        return getToPoint(pose, pathConfig, false); 
                }
                public static Command getToPoint(Pose2d pose, boolean flip){
                        PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(3,4,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),0,0);
                        return getToPoint(pose, pathConfig, flip); 
                }
                public static Command getToPoint(Pose2d pose, PathConfig pathConfig){
                        return getToPoint(pose, pathConfig, false); 
                }
                public static Command getToPoint(double x, double y, double degrees){
                        PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(3,4,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),0,0);
                        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
                        return getToPoint(targetPose,pathConfig, false);
                }
                public static Command getToPoint(double x, double y, double degrees, boolean flip){
                        PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(3,4,Rotation2d.fromDegrees(540),Rotation2d.fromDegrees(540),0,0);
                        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
                        return getToPoint(targetPose,pathConfig, flip);
                }
                public static Command getToPoint(double x, double y, double degrees, PathConfig config){
                        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
                        return getToPoint(targetPose, config, false);
                }
                public static Command getToPoint(double x, double y, double degrees, PathConfig config, boolean flip){
                        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
                        return getToPoint(targetPose, config, flip);
                }
        }
        public static class PathConfig{
                public double maxVelocity = 3;
                public double maxAcceleration = 4;
                public Rotation2d maxAngularVelocity = Rotation2d.fromDegrees(540);
                public Rotation2d maxAngularAcceleration = Rotation2d.fromDegrees(720);
                public double endVelocity = 0;
                public double rotationDelay = 0;
                public PathConfig(){}
                public PathConfig(double maxVelocity, double maxAcceleration, Rotation2d maxAngularVelocity, Rotation2d maxAngularAcceleration, double endVelocity, double rotationDelay){
                        this.maxVelocity = maxVelocity;
                        this.maxAcceleration = maxAcceleration;
                        this.maxAngularVelocity = maxAngularVelocity;
                        this.maxAngularAcceleration = maxAngularAcceleration;
                        this.endVelocity = endVelocity;
                        this.rotationDelay = rotationDelay;
                }
        }
        /**
         * slots out of 5
         * @param config
         * @param slot
         */
        public static void addConfig(PathConfig config, int slot){
                configs[slot] = config;
        }
        /**
         * slots out of 5
         * @param slot
         */
        public static PathConfig getConfig(int slot){
                return configs[slot];
        }
}
