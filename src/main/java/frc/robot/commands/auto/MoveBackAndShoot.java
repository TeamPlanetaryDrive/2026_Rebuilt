package frc.robot.commands.auto;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.bot.ShooterSubsystem;
// import frc.robot.subsystems.drive.DriveSubsystem;

// public class MoveBackAndShoot extends Command {
//     private final DriveSubsystem drive;
//     private final ShooterSubsystem shooter;
//     private Command innerCommand;

//     public MoveBackAndShoot(DriveSubsystem drive, ShooterSubsystem shooter) {
//         this.drive = drive;
//         this.shooter = shooter;
//         addRequirements(drive, shooter);
//     }

//     @Override
//     public void initialize() {
//         Pose2d start = drive.getPose();

//         // Pose2d end = new Pose2d(
//         //     start.getX() - AutoConstants.kMoveBackDistance,
//         //     start.getY(),
//         //     start.getRotation()
//         // );
//         Pose2d end = start.plus(new Transform2d(
//             -AutoConstants.kMoveBackDistance, 0.0, new Rotation2d()
//         ));

//         TrajectoryConfig config = new TrajectoryConfig(
//             AutoConstants.kMaxSpeedMetersPerSecond,
//             AutoConstants.kMaxAccelerationMetersPerSecondSquared
//         )
//             .setKinematics(DriveConstants.kDriveKinematics)
//             .setReversed(true);
            
//         if (start.getTranslation().getDistance(end.getTranslation()) < 0.05) {
//         System.out.println("Auto path too short, skipping move");
//         return;
// }

//         Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//             start,
//             List.of(),
//             end,
//             config
//         );

//         ProfiledPIDController thetaController = new ProfiledPIDController(
//             AutoConstants.kPThetaController,
//             0,
//             0,
//             AutoConstants.kThetaControllerConstraints
//         );
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         Command moveCommand = new SwerveControllerCommand(
//             trajectory,
//             drive::getPose,
//             DriveConstants.kDriveKinematics,
//             new PIDController(AutoConstants.kPXController, 0, 0),
//             new PIDController(AutoConstants.kPYController, 0, 0),
//             thetaController,
//             drive::setModuleStates,
//             drive
//         ).andThen(() -> drive.drive(0, 0, 0, false));

//         innerCommand = Commands.sequence(

//             moveCommand,

//             Commands.runOnce(() -> {
//                 shooter.startShooter();
//                 shooter.feedBackward();
//             }, shooter),
            
//             Commands.waitSeconds(1.0),
//             Commands.runOnce(shooter::feedForward, shooter),

//             Commands.waitSeconds(10.0)


//         );

//         innerCommand.initialize();
//     }

//     @Override
//     public void execute() {
//         if (innerCommand != null) {
//             innerCommand.execute();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // if (innerCommand != null) {
//         //     innerCommand.end(interrupted);
//         // }
//         // // drive.stop();
//         // shooter.stop();
//         return;
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }

import java.util.List;
import java.util.Set;

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

public class MoveBackAndShoot{
    private MoveBackAndShoot() {

    }

    public static Command create(DriveSubsystem drive, ShooterSubsystem shooter) {
    return Commands.defer(() -> {
        drive.disableVisionPoseUpdates();

        Pose2d start = drive.getPose();
        Pose2d end = start.plus(new Transform2d(
            -AutoConstants.kMoveBackDistance, 0.0, new Rotation2d()
        ));

        if (start.getTranslation().getDistance(end.getTranslation()) < 0.05) {
            return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.startShooter();
                    shooter.feedBackward();
                }, shooter),
                Commands.waitSeconds(1.0),
                Commands.runOnce(shooter::feedForward, shooter)
            ).finallyDo(interrupted -> drive.enableVisionPoseUpdates());
        }

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);

        Trajectory trajectory;
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                List.of(),
                end,
                config
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.runOnce(() -> drive.enableVisionPoseUpdates(), drive);
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Command move = new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive
        );

        return Commands.sequence(
            move,
            Commands.runOnce(() -> drive.drive(0, 0, 0, true), drive),
            Commands.runOnce(() -> {
                shooter.startShooter();
                shooter.feedBackward();
            }, shooter),
            Commands.waitSeconds(1.0),
            Commands.runOnce(shooter::feedForward, shooter)
        ).finallyDo(interrupted -> {
            drive.drive(0, 0, 0, true);
            drive.enableVisionPoseUpdates();
        });
    }, Set.of(drive, shooter));
}
}