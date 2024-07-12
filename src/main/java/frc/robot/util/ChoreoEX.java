// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;

/** Add your docs here. */
public class ChoreoEX {
    private static CommandSwerveDrivetrain drivetrain = null;
    /**
     * needed if you want to preset the starting pose, else, will run without preset even if set
     * @param drivebase
     */
    public static void setDrivetrain(CommandSwerveDrivetrain drivebase){
        drivetrain = drivebase;
    }
    public static Command getChoreoPath(boolean presetStart, String pathName){
        PathPlannerPath choreoTraj;
        
        try {
            choreoTraj = PathPlannerPath.fromChoreoTrajectory(pathName);
        } catch (Exception e) {
            System.out.println("\n"+"Choreo path of the name: "+pathName+".traj"+" does not exist in deploy/choreo"+"\n");
            return Commands.none();
        }
        if (presetStart && drivetrain != null){
            return Commands.defer(()->Commands.runOnce(()->{
                if (DriverStation.getAlliance().get() == Alliance.Red){
                    drivetrain.seedFieldRelative(choreoTraj.flipPath().getStartingDifferentialPose());
                    return;
                }
                drivetrain.seedFieldRelative(choreoTraj.getStartingDifferentialPose());
            }),Set.of()).andThen(AutoBuilder.followPath(choreoTraj));
        }else{
            return AutoBuilder.followPath(choreoTraj);
        }
        
        // ChoreoTrajectory traj = Choreo.getTrajectory(pathName); // 

        //
        //  return Choreo.choreoSwerveCommand(
        //     traj, // 
        //     this::getPose, // 
        //     new PIDController(5, 0.0, 0.0), //
        //     new PIDController(5, 0.0, 0.0), //
        //     new PIDController(5, 0.0, 0.0), //
        //     (ChassisSpeeds speeds) -> //
        //         this.setControl(autoRequest.withSpeeds(speeds)),
        //     () -> {
        //         Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        //         boolean mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
        //         return mirror;
        //     }, // 
        //     this // 
        // );
    }
    /**
     * gives choreoPath with no prestart
     * @param name
     * @return
     */
    public static Command getChoreoPath(String name){
        return getChoreoPath(false, name);
    }

    /**
     * will not run if any names are incorrect or not supplied
     * @param presetStart
     * @param names
     * @return
     */
    public static Command getChoreoGroupPath(boolean presetStart, String[] names){
        if (names.length < 1){
            System.out.println("No paths present in groupPath");
            return Commands.none();
        }
        if (getChoreoPath(false, names[0]).getName().equals("InstantCommand")){
            return Commands.none();
        }
        SequentialCommandGroup sequence = new SequentialCommandGroup(Commands.none());
        
        sequence.addCommands(getChoreoPath(presetStart, names[0]));
        
        for (int i = 1; i < names.length; i++){
            Command nextCommand = getChoreoPath(false, names[i]);
            if (nextCommand.getName().equals("InstantCommand")){
                return Commands.none();
            }
            sequence.addCommands(nextCommand);
        }
        return sequence;
    }
}
