package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ClawSpinSubsystem;

public class OutakeClaw extends Command {

    private ClawSpinSubsystem claw;
    final private double spinClawAmount;

    public OutakeClaw(ClawSpinSubsystem claw) {
        spinClawAmount = -1; //TODO: Needs testing
        this.claw = claw;   

        addRequirements(this.claw);
    }

    @Override
    public void initialize() {
        claw.spinClaw(spinClawAmount);
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
