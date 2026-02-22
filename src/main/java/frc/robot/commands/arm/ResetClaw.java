package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class ResetClaw extends Command {
    private final ArmSubsystem arm;


    public ResetClaw(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        arm.resetClawEncoderToBottom();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
    
    }
    //TODO: Added Overrides, test.

}
