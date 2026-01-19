package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.bot.ArmSubsystem;
public class ArmCommand extends Command {

    private ArmSubsystem arm; 
    private double angle;
    private double distance;
    private double clawAngle;
    private int level; 

    public ArmCommand(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        /*
         * 0: Intake
         * 1: L1
         * 2: L2
         * 3: L3
         * 4: L4
         * 5: Climb
         */ 
        level = arm.getArmLevel();
        switch(level) {
            case 0:
                angle = Constants.ArmConstants.intakeAngle;
                distance = Constants.ArmConstants.intakeExtension;
                clawAngle = Constants.ArmConstants.intakeClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "INTAKE");
                break;
            case 1:
                angle = Constants.ArmConstants.l1Angle;
                distance = Constants.ArmConstants.l1Extension;
                clawAngle = Constants.ArmConstants.l1ClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "L1");
                break;
            case 2:
                angle = Constants.ArmConstants.l2Angle;
                distance = Constants.ArmConstants.l2Extension;
                clawAngle = Constants.ArmConstants.l2ClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "L2");
                break;
            case 3:
                angle = Constants.ArmConstants.l3Angle;
                distance = Constants.ArmConstants.l3Extension;
                clawAngle = Constants.ArmConstants.l3ClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "L3");
                break;
            case 4:
                angle = Constants.ArmConstants.l4Angle;
                distance = Constants.ArmConstants.l4Extension;
                clawAngle = Constants.ArmConstants.l4ClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "L4");
                break;
            case 5:
                angle = Constants.ArmConstants.climbAngle;
                distance = Constants.ArmConstants.climbExtension;
                clawAngle = Constants.ArmConstants.climbClawAngle;
                SmartDashboard.putString("CURRENT SETPOINT", "CLIMB");
                break;
            case 6:
                angle = 0;
                distance = 0;
                clawAngle = 0;
                SmartDashboard.putString("CURRENT SETPOINT", "BOTTOM");
                break;
            default:
                angle = 90;
                distance = 0;
                clawAngle = Constants.ArmConstants.beginningClawPosition * 360 / 63;
        }
        arm.rotateArm(angle); 
        arm.extendArm(distance);
        arm.pivotClaw(clawAngle);
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
