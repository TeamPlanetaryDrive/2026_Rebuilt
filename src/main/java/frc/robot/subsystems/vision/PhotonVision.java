package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem; 

public class PhotonVision extends SubsystemBase {
    private DriveSubsystem m_driveSubsystem; 
    private PhotonCamera camera; 
    private AprilTagFieldLayout layout; 
    private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> pose;

    public PhotonVision(String cameraName, DriveSubsystem drive){
        m_driveSubsystem = drive; 
        camera = new PhotonCamera(cameraName);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVisionConstants.transform);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> temp = camera.getAllUnreadResults();
        if(temp.size() > 0) {
            result = temp.get(temp.size()-1);
            if(result.hasTargets())
            {
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
        Optional<Pose3d> pose = layout.getTagPose(fiducialId);
        if(!pose.isPresent()) {
            return null;
        }
        return pose.get();
    }
} 