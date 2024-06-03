package frc.robot;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.LimelightConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic(){

        if (/*DriverStation.isTeleopEnabled() && */!Robot.isSimulation()){
            updateVisionPose(LimelightConstants.LIMELIGHT_NAME);
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0.4;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // getState of the robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0),
                                            new PIDConstants(5, 0, 0),
                    5.3,
            driveBaseRadius,
            new ReplanningConfig()),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                            
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            // Reference to this subsystem to set requirements // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble());
    }
    public Rotation2d getAngleFromPose(Pose2d pose) {
        double deltaX;
        double deltaY;
        deltaX = pose.getX() - getPose().getX();
        deltaY = pose.getY() - getPose().getY();

        double angleRadians = ((Math.atan(deltaY/deltaX)));

        // Convert the angle to Rotation2d
        Rotation2d rotation = Rotation2d.fromRadians(angleRadians - getPose().getRotation().getRadians());
        if (getPose().getX()>pose.getX()){
            if (rotation.getDegrees()>0){
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - getPose().getRotation().getDegrees()-180);
            }else{
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - getPose().getRotation().getDegrees()+180);
            }
        }
        return rotation;
    }
    public Rotation2d getAngleFromCorner() {
        Pose2d pose = new Pose2d(0,8.1026,new Rotation2d());
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            pose = new Pose2d(16.4846,8.1026,new Rotation2d());
        }
        return getAngleFromPose(pose);
    }

    public Rotation2d getPoseAngle(Pose2d pose) {
        double deltaX = pose.getX() - getPose().getX();
        double deltaY = pose.getY() - getPose().getY();
        double angleRadians = ((Math.atan(deltaY/deltaX)));
        return Rotation2d.fromRadians(angleRadians);
    }

    public double getDistanceFromPoseMeters(Pose2d pose) {
        return Math.sqrt(Math.pow(pose.getX()-getPose().getX(),2)+Math.pow(pose.getY()-getPose().getY(),2));
    }

    public void seedFieldRelative(double degrees) {
        try {
            m_stateLock.writeLock().lock();
            m_fieldRelativeOffset = new Rotation2d(Units.degreesToRadians(getState().Pose.getRotation().getDegrees()+degrees));
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }


    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){

        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 16.4846 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.1026;

        return isInBounds;
    }
    private void updateVisionPose(String limelightName){
        double maxRotationalAcceleration = 60;

        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if(botPose.tagCount == 0) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(limelightName, m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if(!isInBouds(botPose)) {
            return;
        }
        if(Math.abs(Units.radiansToDegrees(getState().speeds.omegaRadiansPerSecond)) > maxRotationalAcceleration){

            double[] rotationInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
            addVisionMeasurement(
                botPose.pose,
                botPose.timestampSeconds,VecBuilder.fill(rotationInterpolation[0],rotationInterpolation[1],999999));
            return;
        }
        
        double[] linearInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
        addVisionMeasurement(
            botPose.pose,
            botPose.timestampSeconds,VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],999999));
        
    }
    public Rotation2d getYawOffsetDegrees(){
        return m_fieldRelativeOffset;
    }
    /**
     * all in meters
     * @param pose
     * @param expected
     * @param error
     * @return
     */
    public static boolean poseWithinRange(Pose2d pose, Pose2d expected, double error){
        if (pose.getX()<expected.getX()+error && pose.getX()>expected.getX()-error && pose.getY()<expected.getY()+error && pose.getY()>expected.getY()-error){
            return true;
        }
        return false;
    }
}
    // private double[] currentVisionPose(String limelightName) {
    //     return LimelightHelpers.getBotPose_wpiBlue(limelightName);
    // }

    // private void updateVisionPose(String limelightName) {
    //     // LimelightResults results =  LimelightHelpers.getLatestResults(limelightName);     

    //     if (LimelightHelpers.getFiducialID(limelightName) != -1) {
    //         boolean doRejectUpdate = false;
    //         LimelightHelpers.SetRobotOrientation(limelightName, m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //         LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //         if(Math.abs(m_angularVelocity.getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //         {
    //             doRejectUpdate = true;
    //         }
    //         if(!doRejectUpdate)
    //         {
    //             m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
    //             m_odometry.addVisionMeasurement(
    //                 mt2.pose,
    //                 mt2.timestampSeconds);
    //         }
    //         PoseEstimate botposeEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    //         // System.out.println("Saw Tag");
    //         double[] botpose = currentVisionPose(limelightName);
    //         // System.out.println(getVisionTrust(botpose));
    //         double latency = (Timer.getFPGATimestamp() - (botpose[6]/1000.0));
           
    //         Pose2d currentPose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(Units.degreesToRadians(botpose[5])));
            
    //         // if (Math.abs(currentPose.getX() - getPose().getX()) <= 1 && Math.abs(currentPose.getY() - getPose().getY()) <= 1) {
    //         // System.out.println("Good Data");
    //         double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    //         double targetDistance = Math.sqrt(Math.pow(targetPose[0], 2) + Math.pow(targetPose[1], 2) + Math.pow(targetPose[2], 2));
    //         double[] stddev = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.getLookupValue(targetDistance);
    //         // System.out.println("DistanceFromTarget: " + targetDistance);
    //         double ambiguity = 100;
    //         boolean isInBounds = currentPose.getX() > 0 && currentPose.getX() < 16.4846 && currentPose.getY() > 0 && currentPose.getY() < 8.1026;

    //         try {
    //             ambiguity = botposeEstimate.rawFiducials[0].ambiguity;
    //         } catch (Exception e) {
    //             // TODO: handle exception
    //             // System.out.println("AMBIGUITY BROKEN");
    //         }
    //         // swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stddev[0], stddev[1], Units.degreesToRadians(stddev[2])));
    //         double allowableDist;
    //         // if (DriverStation.isTeleopEnabled()){
    //         allowableDist=6;
    //         // }else{
    //         //     allowableDist=3;
    //         // }
    //         if (botposeEstimate.avgTagDist<allowableDist && ambiguity<.4 && isInBounds){
    //             // System.out.println("Added Vision");
    //             // if (DriverStation.isTeleopEnabled()){
    //                 addVisionMeasurement(currentPose, latency, VecBuilder.fill(stddev[0],stddev[1],stddev[2]));
    //             // }
    //             // else{
    //             //     addVisionMeasurement(currentPose, latency, VecBuilder.fill(stddev[0],stddev[1],10000));
    //             // }
    //         }else{

    //         }

    //         // } else {
    //             // System.out.println("Cannot add vision data - Pose is out of range");
    //         // }
    //         // poseEstimator.addVisionMeasurement(currentPose, latency,VecBuilder.fill(0.9, 0.9, 0.1).times(1.0 / trustWorthiness));
    //     }else{
    //         // System.out.println("No tags present");
    //     }
    // }
    // private PoseEstimate currentVisionPose(String limelightName) {
    //     return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    // }
    // private void updateVisionPose(String limelightName) {
    //     if (LimelightHelpers.getFiducialID(limelightName)!=-1) {
    //         PoseEstimate botpose = currentVisionPose(limelightName);
    //         double latency = botpose.latency;
    //         Pose2d currentPose = botpose.pose;
    //         double avgAmbiguity = 100;
    //         double avgDistance = 4;
    //         for (RawFiducial val : botpose.rawFiducials){
    //             if (val.ambiguity > 0){
    //                 avgAmbiguity += val.ambiguity;
    //             }
    //             avgDistance += val.distToCamera;
    //         }
    //         avgAmbiguity /= botpose.rawFiducials.length;
    //         avgDistance /= botpose.rawFiducials.length;
            // boolean isInRange = currentPose.getX() > getPose().getX()-1 && currentPose.getX() < getPose().getX()+1 && currentPose.getY() > getPose().getY()-1 && currentPose.getY() < getPose().getY()+1;

            // boolean isInBounds = currentPose.getX() > 0 && currentPose.getX() < 16.4846 && currentPose.getY() > 0 && currentPose.getY() < 8.1026;
    //         if (avgAmbiguity < 70 && botpose.avgTagDist < 6 && isInBounds /*&& isInRange*/){
    //             System.out.println("ADDED VISION");
    //             double[] stddev;
    //             if (botpose.tagCount>1){
    //                 stddev = LimelightConstants.TWO_APRIL_TAG_LINEAR_INTERPOLATOR.getLookupValue(avgDistance);
    //             }else{
    //                 stddev = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.getLookupValue(avgDistance);
    //             }
    //             Matrix<N3,N1> stdDeviation = VecBuilder.fill(0,0,0);//stddev[0],stddev[1],stddev[2]);
    //             addVisionMeasurement(currentPose, latency, stdDeviation);
    //         }else{
    //             System.out.println("Could not add vision measurement - out of standard bounds");
    //         }
    //     }else{
    //         System.out.println("Could not add vision measurement - no tags present");
    //     }
    // }