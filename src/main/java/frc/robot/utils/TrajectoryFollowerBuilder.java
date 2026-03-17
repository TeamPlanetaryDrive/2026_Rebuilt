package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class TrajectoryFollowerBuilder {
    private final ProfiledPIDController thetaController;
    private final TrajectoryConfig config;
    private final PhotonVision photonCamera;
    private final DriveSubsystem drive;
    private final Supplier<Pose2d> poseProvider;
    
    public TrajectoryFollowerBuilder(DriveSubsystem drive, PhotonVision photonCamera, Supplier<Pose2d> poseProvider) {
        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.drive = drive;
        this.photonCamera = photonCamera;
        this.poseProvider = poseProvider;
        config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);
    }
    
    public Command buildSwerveController(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand (
            trajectory, 
            drive::getPose, 
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive
        );

        drive.resetOdometry(trajectory.getInitialPose());

        return command.andThen(() -> drive.drive(0, 0, 0, false)); 
    }

    public Command alignToAprilTag(Pose2d currentPose, Pose2d aprilTagPose) {
        Trajectory trajectory;
        Pose2d endingPose;
        double distance;
        //redo for small code statement
        String currentLevel = SmartDashboard.getString("CURRENT SETPOINT", "L4");
        // contains loop for boolean
        String[] arr = new String[]{"INTAKE", "CLIMB", "L1", "L2", "L3", "L4"};
        boolean contains = false;
        for (String s : arr) {
            if (s.equals(currentLevel)) {
                contains = true;
                break;
            }
        }
        if (currentLevel.equals("INTAKE")) {
            distance = .2413;
        } else if(contains) {
            distance = .25;
        } else {
            return null;
        }

        double angle = aprilTagPose.getRotation().getRadians();
        double x = aprilTagPose.getX();
        double y = aprilTagPose.getY();

        x += distance * Math.cos(angle);
        y += distance * Math.sin(angle);

        endingPose = new Pose2d(x, y, new Rotation2d(angle));

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(currentPose);
        poses.add(endingPose);

        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        return buildSwerveController(trajectory);
    }

    public Command buildTrajectoryFromPosesList(List<Pose2d> poses) {
        return buildSwerveController(TrajectoryGenerator.generateTrajectory(poses, config));
    }

    public Command alignToL4() {
        SmartDashboard.putString("CURRENT SETPOINT", "L4");
        return alignToAprilTag(
            poseProvider.get(),
            photonCamera.getAprilTagPose(
                new int[] {6, 8, 9, 11, 17, 19, 20, 22}  
            )
        );
    }
}
