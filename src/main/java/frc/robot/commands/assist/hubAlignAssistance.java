package frc.robot.commands.assist;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.bot.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubAlignAssistance extends Command {
    private final DriveSubsystem drive;
    private final PhotonVision vision;
    private final PIDController distancePid;

    private double lastValidTagTime = -1.0;

    public HubAlignAssistance(DriveSubsystem drive, PhotonVision vision) {
        this.drive = drive;
        this.vision = vision;

        this.distancePid = new PIDController(
            Constants.DriveConstants.kP,
            Constants.DriveConstants.kI,
            Constants.DriveConstants.kD
        );

        distancePid.setTolerance(Constants.DriveConstants.kDistanceTolerance);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        distancePid.reset();

        OptionalDouble distanceOpt = vision.getTargetTagDistanceMeters();
        if (distanceOpt.isPresent()) {
            lastValidTagTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else {
            lastValidTagTime = -1.0;
        }

        // SmartDashboard.putString("Vision Align Command", "Initialized");
    }

    @Override
    public void execute() {
        OptionalDouble distanceOpt = vision.getTargetTagDistanceMeters();

        if (distanceOpt.isPresent()) {
            lastValidTagTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

            double currentDistance = distanceOpt.getAsDouble();
            double output = distancePid.calculate(
                currentDistance,
                Constants.AutoConstants.kMoveBackDistance
            );

            output = MathUtil.clamp(
                output,
                -Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxSpeedMetersPerSecond
            );

            SmartDashboard.putNumber("Tag Distance", currentDistance);
            SmartDashboard.putNumber(
                "Tag Distance Error",
                currentDistance - Constants.AutoConstants.kMoveBackDistance
            );
            SmartDashboard.putNumber("Tag PID Output", output);

            // forward/backward only; no driver input
            drive.drive(output, 0.0, 0.0, true);
        } else {
            // lost tag temporarily: stop while waiting briefly
            drive.drive(0.0, 0.0, 0.0, true);
            SmartDashboard.putString("Vision Align Command", "Tag temporarily lost");
        }
    }

    @Override
    public boolean isFinished() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // cancel if no recent tag was available at all
        if (lastValidTagTime < 0) {
            return true;
        }

        // end if tag has been lost too long
        if ((now - lastValidTagTime) > Constants.DriveConstants.kLostTagCancelSec) {
            return true;
        }

        // end if PID is satisfied and we currently still have a distance reading
        OptionalDouble distanceOpt = vision.getTargetTagDistanceMeters();
        if (distanceOpt.isPresent()) {
            return distancePid.atSetpoint();
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, true);

        if (interrupted) {
            SmartDashboard.putString("Vision Align Command", "Interrupted");
        } else {
            SmartDashboard.putString("Vision Align Command", "Finished");
        }
    }
}