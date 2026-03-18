package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;

public class AlignToL3 extends Command{
    DriveSubsystem drive;
    PhotonVision photonvision;


    public AlignToL3(DriveSubsystem drive, PhotonVision photonvision) {
        this.drive = drive;
        this.photonvision = photonvision;
    }
    @Override
    public void initialize() {
        Pose2d pose = drive.getPose();
        Pose2d aprilTagPose = photonvision.getAprilTagPose(
                new int[] {6, 8, 9, 11, 17, 19, 20, 22}  
        );
        if(aprilTagPose == null) {
            System.out.println("No apriltag detected by the camera");
            return;
        }
        Trajectory trajectory;
        Pose2d endingPose;
        double distance = 0.22606;
        double angle = aprilTagPose.getRotation().getRadians();
        double x = aprilTagPose.getX();
        double y = aprilTagPose.getY();

        x += distance * Math.cos(angle);
        y += distance * Math.sin(angle);

        endingPose = new Pose2d(x, y, new Rotation2d(angle));

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(pose);
        poses.add(endingPose);

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);

        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

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

        CommandScheduler.getInstance().schedule(command.andThen(() -> drive.drive(0, 0, 0, false)));

    }
    @Override
    public void execute() {}
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {}
}
