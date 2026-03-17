package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Constants;

public class AlignToAprilTag extends Command {

    // TO DO:
    // GET THE CAMERA POSITION CONSTANT FIGURED OUT IN CONSTANTS.JAVA

    // technically we could just use the trajectory thing but i dont wanna do allat

    // constraints/motion profiles for movement
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

    // 3 seperate controllers for x, y, and rot. control
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS); // same as theta

    // basic modules
    private final PhotonCamera photonCamera;
    private final DriveSubsystem drivetrain;
    private final Supplier<Pose2d> poseProvider;

    // our goal pose relative to the tag; 1.5 meters away facing it directly head on 
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));
    
    private PhotonTrackedTarget prevTarget;

    public AlignToAprilTag(PhotonCamera photonCamera, DriveSubsystem drivetrain, Supplier<Pose2d> poseProvider) {
        this.photonCamera = photonCamera;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    } 

    @Override
    public void initialize() {
        prevTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = 
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0, 
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        
        // new feature -- getAllUnreadResults; getLatestResult is depricated 
        var photonRes = photonCamera.getAllUnreadResults().get(-1);
        if (photonRes.hasTargets()) {
            // find the tag we want to chase; ngl i dont think this is necessary because of getAllUnreadResults, but i don't know if it works the way it does so i'll 
            // leave !t.equals(prevTarget) in for now
            var targetOpt = photonRes.getTargets().stream().filter(t -> !t.equals(prevTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1);

            if (targetOpt.findFirst().isPresent()) {
                var target = targetOpt.findFirst().get();

                // this is new target data, so recalculate the goal for the next iter
                prevTarget = target;
                
                // get camera pose by transforming the robot pose by a known offset
                var cameraPose = robotPose.transformBy(Constants.VisionConstants.cameraPositionRobotRel);

                // describes position of target relative to camera -> returns transformation which maps to apriltag
                var camToTarget = target.getBestCameraToTarget();

                // finds target pose by transforming the camera pose by the position of the target relative to the camera
                var targetPose = cameraPose.transformBy(camToTarget);
                
                // then we transform the target pose by the transform we def. earlier to get our goal pose
                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                // drive to goal pose
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());
            }
        }
        
        if (prevTarget == null) {
            // No target has been visible
            drivetrain.stop();
        } else {
            // Drive to the target
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
