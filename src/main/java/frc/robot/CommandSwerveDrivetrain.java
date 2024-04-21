package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.security.cert.TrustAnchor;
import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.LimelightConstants;
import frc.robot.util.MultiLinearInterpolator;
// import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Vision;
import frc.robot.util.ModifiedSignalLogger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double lastTime = DriverStation.getMatchTime();
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
        // System.out.println(getDistanceFromPieceVertical());
        // if (isPiecePresent()){
        //     System.out.println(getPiecePose().get());
        // }
        // System.out.println(Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-object")));

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
        Pose2d tagLocation = new Pose2d(0,8.1026,new Rotation2d());
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            tagLocation = new Pose2d(16.4846,8.1026,new Rotation2d());
        }
        return getAngleFromPose(tagLocation);
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
    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){

        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 16.4846 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.1026;

        return isInBounds;
    }
    private void updateVisionPose(String limelightName){
        double maxRotationalAcceleration = 60;
        // double currentTime = Timer.getFPGATimestamp();

        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if(botPose.tagCount == 0) {
            return;
        }
                //  System.out.println(Timer.getFPGATimestamp() - currentTime);
        LimelightHelpers.SetRobotOrientation(limelightName, m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if(Math.abs(Units.radiansToDegrees(getState().speeds.omegaRadiansPerSecond)) > maxRotationalAcceleration) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            return;
        }
        
        if(!isInBouds(botPose)) {
            // System.out.println("Not in bounds");
            return;
        }
        addVisionMeasurement(
            botPose.pose,
            botPose.timestampSeconds,VecBuilder.fill(.6,.6,999999));
        
    }
    public Rotation2d getYawOffsetDegrees(){
        return m_fieldRelativeOffset;
    }
    public Rotation2d getRotationFromPiece(String limelightName){
        return Rotation2d.fromDegrees(-LimelightHelpers.getTX(limelightName));        
    }
    public boolean isPiecePresent(){
        if (LimelightHelpers.getTA(LimelightConstants.AMP_CAM) != 0){
            return true;
        }
        return false;
    }
    public double getDistanceFromPieceVertical(){
        if (LimelightHelpers.getTA(LimelightConstants.AMP_CAM) != 0){
            NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.AMP_CAM);
            NetworkTableEntry ty = table.getEntry("ty");
            double targetOffsetAngle_Vertical = ty.getDouble(0.0);
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = Units.radiansToDegrees(LimelightConstants.AMP_CAM_TRANSFORM.getRotation().getY()); 

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = Units.metersToInches(LimelightConstants.AMP_CAM_TRANSFORM.getZ()); 

            // distance from the target to the floor
            double goalHeightInches = 1; 

            double angleToGoalDegrees = -limelightMountAngleDegrees + targetOffsetAngle_Vertical;

            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            return  Units.inchesToMeters((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians))+LimelightConstants.AMP_CAM_TRANSFORM.getY();
        }
        return 0;
    }
    public double getDistanceFromPieceHorizontal(){
        if (LimelightHelpers.getTA(LimelightConstants.AMP_CAM) != 0){
            NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.AMP_CAM);
            NetworkTableEntry tx = table.getEntry("tx");
            double targetOffsetAngle_Vertical = tx.getDouble(0.0);
            
            // what is the yaw of your limelight
            double limelightMountAngleDegrees = Units.radiansToDegrees(LimelightConstants.AMP_CAM_TRANSFORM.getRotation().getZ()); 

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            return getDistanceFromPieceVertical()*Math.tan(angleToGoalRadians);//+LimelightConstants.AMP_CAM_TRANSFORM.getX();
        }
        return 0;
    }
    // public Optional<Pose2d> getPiecePose(){
    //     if (LimelightHelpers.getTA(LimelightConstants.AMP_CAM) != 0){
    //         double x = getDistanceFromPieceVertical();
    //         double y = getDistanceFromPieceHorizontal();
    //         double xn = x*Math.cos(getPose().getRotation().getRadians())- y*Math.sin(getPose().getRotation().getRadians());
    //         double yn = x*Math.sin(getPose().getRotation().getRadians())+ y*Math.cos(getPose().getRotation().getRadians());
    //         return Optional.of(new Pose2d(xn+getPose().getX(),yn+getPose().getY(),new Rotation2d()));
    //     }
    //     return null;
    // }
    // /**
    //  * all in meters
    //  * @param pose
    //  * @param expected
    //  * @param error
    //  * @return
    //  */
    // public static boolean poseWithinRange(Pose2d pose, Pose2d expected, double error){
    //     if (pose.getX()<expected.getX()+error && pose.getX()>expected.getX()-error && pose.getY()<expected.getY()+error && pose.getY()>expected.getY()-error){
    //         return true;
    //     }
    //     return false;
    // }
}
