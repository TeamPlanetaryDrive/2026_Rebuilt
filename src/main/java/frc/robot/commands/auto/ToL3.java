package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.bot.ArmSubsystem;

public class ToL3 extends Command {

    private ArmSubsystem arm; 
    private double angle;
    private double distance;
    private double clawAngle; 

    public ToL3(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        angle = Constants.ArmConstants.l3Angle;
        distance = Constants.ArmConstants.l3Extension;
        clawAngle = Constants.ArmConstants.l3ClawAngle;
        SmartDashboard.putString("CURRENT SETPOINT", "L4");
        
        arm.rotateArm(angle); 
        arm.extendArm(distance);
        arm.pivotClaw(clawAngle);
        arm.setArmLevel(4);
    }
    
    @Override
    public void execute() {}
    
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getPivotSpeed()) < 0.25;
    }

    @Override
    public void end(boolean interrupted) {}
}
