package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.ArmSubsystem;

public class GoUp extends Command {
    private final ArmSubsystem arm;

    public GoUp(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        double angle = Math.abs(arm.getArmLevelDegrees())+0.01;
        arm.rotateArm(angle*360);
    }
    @Override
    public void execute() {}
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    //TODO: Added Overrides, test.
    public void end(boolean interrupted) {}


}
