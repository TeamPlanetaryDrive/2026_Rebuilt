package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy; not used
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem; 

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera; 
    private final AprilTagFieldLayout layout; 
    private final PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> pose;

    public PhotonVision(String cameraName){
        camera = new PhotonCamera(cameraName);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        poseEstimator = new PhotonPoseEstimator(layout, PhotonVisionConstants.transform);
        //PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, was in between layout and PhotoVisionConstants.transform
    }

    @Override
    public void periodic() {
        // List<PhotonPipelineResult> temp = camera.getAllUnreadResults();
        // if(!temp.isEmpty()) {
        //     result = temp.get(temp.size()-1);
        //     if(result.hasTargets())
        //     {
        //         pose = poseEstimator.update(result);
        //         m_driveSubsystem.addVision(pose.get());
        //     } 
        // }
        return;
    }

    public void update(DriveSubsystem drive){
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results){
            if (!result.hasTargets()){
                continue;
            }
            Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (estimate.isEmpty()) {
                continue;
            }

            drive.m_poseEstimator.addVisionMeasurement(
                estimate.get().estimatedPose.toPose2d(),
                estimate.get().timestampSeconds,
                getVisionStdDevs(result));
        }
    }

    private Matrix<N3, N1> getVisionStdDevs(PhotonPipelineResult result) {
    // Very simple placeholder heuristic. Tune on your robot.
    int tagCount = result.getTargets().size();
    double bestTagDistanceMeters =
        result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

    if (tagCount >= 2) {
      // Multi-tag: trust more
      return VecBuilder.fill(0.20, 0.20, Units.degreesToRadians(8.0));
    }

    if (bestTagDistanceMeters < 2.5) {
      // One close tag: medium trust
      return VecBuilder.fill(0.45, 0.45, Units.degreesToRadians(20.0));
    }

    // One far tag: low trust
    return VecBuilder.fill(1.50, 1.50, Units.degreesToRadians(999.0));
  }

    public PhotonCamera getCamera() {
        return camera;
    }

    /*
     * @brief Get the pose of the april tag we want relative to the robot
     * @param ids: array of ints feducial ids for targets we want to match with
     * @return Returns pose of april tag relative to the robot camera as Pose2d
     */
    public Pose2d getAprilTagPose(int[] ids){

        if(result != null &&result.hasTargets()){
            List<PhotonTrackedTarget> targets = result.getTargets();
            int id = -1;

            for (PhotonTrackedTarget tag : targets) {

                for (int i = 0; i < ids.length; i++)
                {
                    if(tag.getFiducialId() == ids[i])
                    {
                        id = tag.getFiducialId(); 
                        break; 
                    }
                }
            }

            if (id == -1)
            {
                return null;
            }

            return getPoseById(id).toPose2d(); 
        }

        return null;
    }

    public Pose3d getPoseById(int fiducialId) {
        Optional<Pose3d> poseEx = layout.getTagPose(fiducialId);
        if(!poseEx.isPresent()) {
            return null;
        }
        return poseEx.get();
    }
} 