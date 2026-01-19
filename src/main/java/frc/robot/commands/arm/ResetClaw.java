package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class ResetClaw extends Command {
    private ArmSubsystem arm;


    public ResetClaw(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    public void initialize() {
        
    }

    public void execute() {
        arm.resetClawEncoderToBottom();
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    
    }


}
