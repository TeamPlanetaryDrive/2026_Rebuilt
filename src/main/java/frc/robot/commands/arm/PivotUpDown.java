package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class PivotUpDown extends Command {
    private final ArmSubsystem arm;
    double speed;

    public PivotUpDown(ArmSubsystem arm, double speed) {
        this.arm = arm;
        addRequirements(arm);
        this.speed = speed;
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        arm.pivotClawDutyCycle(speed);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        arm.pivotClawDutyCycle(0);
        arm.setClawPositionHere();
    }
    //TODO: Added Overrides, test.

}
