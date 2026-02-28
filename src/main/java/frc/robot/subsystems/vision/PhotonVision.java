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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem; 

public class PhotonVision extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem; 
    private final PhotonCamera camera; 
    private final AprilTagFieldLayout layout; 
    private final PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> pose;

    public PhotonVision(String cameraName, DriveSubsystem drive){
        m_driveSubsystem = drive; 
        camera = new PhotonCamera(cameraName);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        poseEstimator = new PhotonPoseEstimator(layout, PhotonVisionConstants.transform);
        //PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, was in between layout and PhotoVisionConstants.transform
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> temp = camera.getAllUnreadResults();
        if(!temp.isEmpty()) {
            result = temp.get(temp.size()-1);
            if(result.hasTargets())
            {
                /* TODO: Figure out how to update() without deprecated method. We can probably use these but I don't know the exacts:
                .estimatePnpDistanceTrigSolvePose(PhotonPipelineResult cameraResult)
                .estimateConstrainedSolvepnpPose(
                                                PhotonPipelineResult cameraResult,
                                                Matrix<N3,N3> cameraMatrix,
                                                Matrix<N8,N1> distCoeffs,
                                                Pose3d seedPose,
                                                boolean headingFree,
                                                double headingScaleFactor)
                .estimateCoprocMultiTagPose(PhotonPipelineResult cameraResult)
                .estimateRioMultiTagPose(
                                        PhotonPipelineResult cameraResult,
                                        Matrix<N3,N3> cameraMatrix,
                                        Matrix<N8,N1> distCoeffs)
                .estimateLowestAmbiguityPose(PhotonPipelineResult cameraResult)
                .estimateClosestToCameraHeightPose(PhotonPipelineResult cameraResult)
                .estimateClosestToReferencePose(
                                                PhotonPipelineResult cameraResult,
                                                Pose3d referencePose)
                .estimateAverageBestTargetsPose(PhotonPipelineResult cameraResult)
                .estimatePnpDistanceTrigSolvePose(PhotonPipelineResult cameraResult)
                .estimateRioMultiTagPose(
                                        PhotonPipelineResult cameraResult,
                                        Matrix<N3,N3> cameraMatrix,
                                        Matrix<N8,N1> distCoeffs)
                */
                pose = poseEstimator.update(result);
                m_driveSubsystem.addVision(pose.get());
            } 
        }
        
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