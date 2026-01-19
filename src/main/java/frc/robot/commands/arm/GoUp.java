package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class GoUp extends Command {
    private ArmSubsystem arm;

    public GoUp(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    public void initialize() {
        double angle = Math.abs(arm.getArmLevelDegrees())+0.01;
        arm.rotateArm(angle*360);
    }

    public void execute() {}

    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {}


}
