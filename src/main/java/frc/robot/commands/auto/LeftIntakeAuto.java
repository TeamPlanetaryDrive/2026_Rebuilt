package frc.robot.commands.auto;

import java.util.List;
// import java.util.List;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.ProfiledPIDController;


import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.bot.RelativeIntakeSubsystem;
import frc.robot.subsystems.bot.ShooterSubsystem;

public class LeftIntakeAuto {
    private LeftIntakeAuto() {}

    public static Command create(DriveSubsystem drive, RelativeIntakeSubsystem intake, ShooterSubsystem shooter) {
        // Lower the intake (angle-only), then stop rotating the arm
        Command lowerIntake = Commands.startEnd(
            () -> intake.setRotateSpeed(-40, 40),
            intake::coastIntakeRotate,
            intake
        ).withTimeout(0.75);

        // 2) Turn 90 degrees to the right (clockwise)
        Command turnRight90 = turnByDegrees(drive, -90.0);

        // 3) Shoot for 3 seconds (spin up, brief prefeed reverse, then feed forward)
        Command shoot3s = Commands.sequence(
            Commands.runOnce(shooter::startShooter, shooter),
            // Commands.waitSeconds(0.35),
            Commands.runOnce(shooter::feedBackward, shooter),
            Commands.waitSeconds(1.0),
            Commands.runOnce(shooter::feedForward, shooter),
            Commands.waitSeconds(3.0)
        ).andThen(Commands.runOnce(shooter::stop, shooter));

        // 4) Turn 90 degrees to the left (counter-clockwise)
        Command turnLeft90 = turnByDegrees(drive, 90.0);

        // 5) Drive forward kMiddleIntakeDistance along current heading
        Command driveToMiddle = driveForwardMeters(drive, AutoConstants.kMiddleIntakeDistance);

        // 6) Turn 90 degrees to the right again
        Command turnRightAgain = turnRight90;

        // 7) Drive forward slowly for 10 seconds while intaking
        double forwardNorm = MathUtil.clamp(
            AutoConstants.kMaxIntakeVelocity / DriveConstants.kMaxSpeedMetersPerSecond,
            -1.0,
            1.0
        );
        Command slowDriveWithIntake = Commands.deadline(
            Commands.waitSeconds(AutoConstants.kMaxIntakeTime),
            Commands.run(() -> drive.drive(forwardNorm, 0.0, 0.0, false), drive),
            Commands.startEnd(intake::start, intake::stop, intake)
        ).andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive));

        return Commands.sequence(
            lowerIntake,
            turnRight90,
            shoot3s,
            turnLeft90,
            driveToMiddle,
            turnRightAgain,
            slowDriveWithIntake
        );
    }

    private static Command turnByDegrees(DriveSubsystem drive, double degrees) {
        return Commands.defer(() -> {
            double target = drive.getRotation().getRadians() + Math.toRadians(degrees);
            var thetaController = new ProfiledPIDController(
                DriveConstants.hP, DriveConstants.hI, DriveConstants.hD, AutoConstants.kThetaControllerConstraints
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            return Commands.run(
                () -> {
                    double omegaRps = MathUtil.clamp(
                        thetaController.calculate(drive.getRotation().getRadians(), target),
                        -DriveConstants.kMaxAngularSpeed,
                         DriveConstants.kMaxAngularSpeed
                    );
                    double rotCmd = omegaRps / DriveConstants.kMaxAngularSpeed;
                    drive.drive(0.0, 0.0, rotCmd, false);
                },
                drive
            ).until(thetaController::atSetpoint)
             .andThen(() -> drive.drive(0, 0, 0, false));
        }, Set.of(drive));
    }

    private static Command driveForwardMeters(DriveSubsystem drive, double meters) {
        return Commands.defer(() -> {
            Pose2d start = drive.getPose();
            // Move forward along current heading
            Pose2d end = start.transformBy(new Transform2d(new Translation2d(meters, 0.0), new Rotation2d()));

            TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            ).setKinematics(DriveConstants.kDriveKinematics);

            Trajectory traj = TrajectoryGenerator.generateTrajectory(List.of(start, end), config);
            return drive.getSwerveControllerCommand(traj);
        }, Set.of(drive));
    }
}
