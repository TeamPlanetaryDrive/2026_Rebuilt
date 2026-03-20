package frc.robot.commands.assist;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;


//  Command that assists the driver in aligning to the hub using vision data
// first should be used to move back to a set distance from the hub
// no driver control input; robot drives forward/backward to reach target distance
// ends when at target distance or vision target lost for too long
public class hubAlignAssistance extends Command {
    private final DriveSubsystem drive;
    private final PhotonVision vision;
    private final PIDController distancePid;

    private double lastValidTagTime = -1.0;

    public hubAlignAssistance(DriveSubsystem drive, PhotonVision vision) {
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

            // output = MathUtil.clamp(
            //     output,
            //     -Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            //     Constants.AutoConstants.kMaxSpeedMetersPerSecond
            // );

            double speedCmdMps = MathUtil.clamp(
            distancePid.calculate(
            currentDistance,
            Constants.AutoConstants.kMoveBackDistance
            ),
            -Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxSpeedMetersPerSecond
            );

            double xCmd = speedCmdMps / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

            SmartDashboard.putNumber("Tag Distance", currentDistance);
            SmartDashboard.putNumber("Speed", xCmd);
            SmartDashboard.putNumber(
                "Tag Distance Error",
                currentDistance - Constants.AutoConstants.kMoveBackDistance
            );
            SmartDashboard.putNumber("Tag PID Output", speedCmdMps);

            // forward/backward only; no driver input
            drive.drive(-xCmd, 0.0, 0.0, false);
        } else {
            // lost tag temporarily: stop while waiting briefly
            drive.drive(0.0, 0.0, 0.0, false);
            SmartDashboard.putString("Vision Align Command", "Tag temporarily lost");
        }
    }

    @Override
    public boolean isFinished() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // cancel if no recent tag was available at all
        if (lastValidTagTime < 0) {
            SmartDashboard.putString("Vision Align Status", "no recent available tag");
            return true;
        }

        // end if tag has been lost too long
        if ((now - lastValidTagTime) > Constants.DriveConstants.kLostTagCancelSec) {
            SmartDashboard.putString("Vision Align Status", "Tag has been lost!!!");
            return true;
        }

        // end if PID is satisfied and we currently still have a distance reading
        OptionalDouble distanceOpt = vision.getTargetTagDistanceMeters();
        if (distanceOpt.isPresent()) {
            SmartDashboard.putString("Vision Align Status", "reached PID location w/ distance");
            return distancePid.atSetpoint();
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, false);

        if (interrupted) {
            SmartDashboard.putString("Vision Align Command", "Interrupted");
        } else {
            SmartDashboard.putString("Vision Align Command", "Finished");
        }
    }
}