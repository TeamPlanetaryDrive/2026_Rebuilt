package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.bot.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class MoveBackAndShoot extends Command {
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private Command innerCommand;

    public MoveBackAndShoot(DriveSubsystem drive, ShooterSubsystem shooter) {
        this.drive = drive;
        this.shooter = shooter;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        // Disable vision-based pose injections during this autonomous
        drive.disableVisionPoseUpdates();
        Pose2d start = drive.getPose();

        
        drive.resetOdometry(start);
        // create endpoint
        Pose2d end = start.transformBy(new Transform2d(-AutoConstants.kMoveBackDistance, 0.0, new Rotation2d()));


        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics);

        // generate the trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(start, end),
            config
        );


        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0.001,
            AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // set
        Command moveCommand = new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0.001),
            new PIDController(AutoConstants.kPYController, 0, 0.001),
            thetaController,
            drive::setModuleStates,
            drive
        ).andThen(() -> drive.drive(0, 0, 0, true));

        innerCommand = Commands.sequence(

            moveCommand,

            Commands.runOnce(() -> {
                shooter.startShooter();
                shooter.feedBackward();
            }, shooter),
            
            Commands.waitSeconds(1.0),
            Commands.runOnce(shooter::feedForward, shooter),

            Commands.waitSeconds(5.0)


        );

        innerCommand.initialize();
    }

    @Override
    public void execute() {
        if (innerCommand != null) {
            innerCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Re-enable vision pose updates when this auto ends
        drive.enableVisionPoseUpdates();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
