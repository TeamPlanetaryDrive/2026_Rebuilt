package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.bot.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ShooterSubsystem shooter) {
        addCommands(
            Commands.runOnce(shooter::startShooter, shooter),
            Commands.waitSeconds(0.1),   // tune shooter spinup time

            Commands.runOnce(shooter::feedForward, shooter),
            Commands.waitSeconds(0.1),   // tune feed time

            Commands.runOnce(shooter::stop, shooter)
        );
    }
}