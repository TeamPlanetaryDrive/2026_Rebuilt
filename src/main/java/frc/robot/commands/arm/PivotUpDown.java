package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class PivotUpDown extends Command {
    private ArmSubsystem arm;
    double speed;

    public PivotUpDown(ArmSubsystem arm, double speed) {
        this.arm = arm;
        addRequirements(arm);
        this.speed = speed;
    }

    public void initialize() {
        
    }

    public void execute() {
        arm.pivotClawDutyCycle(speed);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        arm.pivotClawDutyCycle(0);
        arm.setClawPositionHere();
    }


}
