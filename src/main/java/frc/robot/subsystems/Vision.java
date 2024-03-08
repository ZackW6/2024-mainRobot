// /*
//  * MIT License
//  *
//  * Copyright (c) PhotonVision
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */


package frc.robot.subsystems;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

//import static frc.robot.Constants.Vision.*;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Telemetry;
import frc.robot.Constants.VisionConstants;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;


public class Vision extends SubsystemBase{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;


    // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;


    public Vision(String cameraName, Transform3d transformation) {
        VisionConstants.kTagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        camera = new PhotonCamera(cameraName);
       
        photonEstimator =
                new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, transformation);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        // ----- Simulation
        // if (Robot.isSimulation()) {
        //     // Create the vision system simulation which handles cameras and targets on the field.
        //     // visionSim = new VisionSystemSim("main");
        //     // // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        //     // visionSim.addAprilTags(VisionConstants.kTagLayout);
        //     // Create simulated camera properties. These can be set to mimic your actual camera.
        //     var cameraProp = new SimCameraProperties();
        //     cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        //     cameraProp.setCalibError(0.35, 0.10);
        //     cameraProp.setFPS(15);
        //     cameraProp.setAvgLatencyMs(50);
        //     cameraProp.setLatencyStdDevMs(15);
        //     // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        //     // targets.
        //     cameraSim = new PhotonCameraSim(camera, cameraProp);
        //     // Add the simulated camera to view the targets on this simulated field.
        //     visionSim.addCamera(cameraSim, robotToCam);


        //     cameraSim.enableDrawWireframe(true);
        // }
    }


    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }


    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }


    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return VecBuilder.fill(estStdDevs.get(0, 0), estStdDevs.get(1,0), 100);
    }
// fill(0,0,0);

    // ----- Simulation


    // public void simulationPeriodic(Pose2d robotSimPose) {
    //     visionSim.update(robotSimPose);
    // }


    /** Reset pose history of the robot in the vision system simulation. */
    // public void resetSimPose(Pose2d pose) {
    //     if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    // }


    // /** A Field2d for visualizing our robot and objects on the field. */
    // public Field2d getSimDebugField() {
    //     if (!Robot.isSimulation()) return null;
    //     return visionSim.getDebugField();
    // }
    public boolean hasTarget(){
        var result = camera.getLatestResult();
        return result.hasTargets();
    }
    /**
     * returns the angle from the target if visible, else it returns 361
     * @param target
     * @return
     */
    public double getTargetAngle(int target){
        var result = camera.getLatestResult();
        if (result.hasTargets()){
            List<PhotonTrackedTarget> targets = result.getTargets();
            var foundTargets = targets.stream().filter(t -> t.getFiducialId()==9).filter(t ->!t.equals(9) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() !=-1).findFirst();
            if (foundTargets.isPresent()){
                System.out.println(foundTargets.get().getYaw());
                return foundTargets.get().getYaw();
            }
        }
        return 0;
    }
    /**
     * returns the meters from the target if visible, else it returns 361
     * @param target
     * @return
     */
    public double getTargetDist(int target){
        var result = camera.getLatestResult();
        if (result.hasTargets()){
            List<PhotonTrackedTarget> targets = result.getTargets();
            var foundTargets = targets.stream().filter(t -> t.getFiducialId()==4).filter(t ->!t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() !=-1).findFirst();
            if (foundTargets.isPresent()){
                return PhotonUtils.calculateDistanceToTargetMeters(
                    VisionConstants.ShooterCamTransform.getX(),
                    VisionConstants.ShooterCamTransform.getY(),
                    VisionConstants.ShooterCamTransform.getZ(),
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            }
        }
        return 0;
    }
    // public void logTelemetry(Telemetry telemetry){
    //     telemetry.registerVisionTelemetry(getEstimatedGlobalPose().get().estimatedPose.toPose2d());
    // }
}
