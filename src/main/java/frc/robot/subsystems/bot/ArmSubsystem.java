package frc.robot.subsystems.bot;

// PersistMode and ResetMode were originally in com.revrobotics.spark.SparkBase, but that has been deprecated.
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    // motors
    private final SparkMax armExtend;
    private final SparkMax armPivotL; 
    private final SparkMax armPivotR;
    private final SparkMax clawPivot;
    // PID controllers
    private final SparkClosedLoopController armExtendPID; 
    private final SparkClosedLoopController armPivotPID; 
    private final SparkClosedLoopController clawPivotPID;
    // encoders
    private final RelativeEncoder armExtendEncoder; 
    private final RelativeEncoder armPivotLEncoder; 
    private final AbsoluteEncoder armPivotREncoder;
    private final RelativeEncoder clawPivotEncoder;

    private int currentLevel = 0;
    //private HashMap<Integer, String> map;
    // TODO: Remove this line if you agree to use the String array instead of the HashMap.
    private final String[] levels = {"INTAKE", "L1", "L2", "L3", "L4", "CLIMB", "BOTTOM", "VERTICAL"};

    public ArmSubsystem() {
        armExtend = new SparkMax(Constants.ArmConstants.armExtendCANID, SparkLowLevel.MotorType.kBrushless);
        armPivotL = new SparkMax(Constants.ArmConstants.armPivotLCANID, SparkLowLevel.MotorType.kBrushless);
        armPivotR = new SparkMax(Constants.ArmConstants.armPivotRCANID, SparkLowLevel.MotorType.kBrushless);
        clawPivot = new SparkMax(Constants.ArmConstants.clawPivotCANID, SparkLowLevel.MotorType.kBrushless);

        armExtend.configure(Configs.ArmSubsystem.extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armPivotL.configure(Configs.ArmSubsystem.pivotLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armPivotR.configure(Configs.ArmSubsystem.pivotRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clawPivot.configure(Configs.ArmSubsystem.clawPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armExtendEncoder = armExtend.getEncoder(); 
        armPivotLEncoder = armPivotL.getEncoder(); 
        armPivotREncoder = armPivotR.getAbsoluteEncoder();
        clawPivotEncoder = clawPivot.getEncoder();

        armExtendPID = armExtend.getClosedLoopController(); 
        armPivotPID = armPivotR.getClosedLoopController(); 
        clawPivotPID = clawPivot.getClosedLoopController();

        configEncoders();

        SmartDashboard.putString("SELECTED LEVEL", levels[currentLevel]);
    }
    public final void configEncoders() {
        armExtendEncoder.setPosition(0);
        clawPivotEncoder.setPosition(Constants.ArmConstants.beginningClawPosition);
    }
    
    public void extendArm(double distance) {
        double outputRotations = distance * Constants.ArmConstants.armExtensionConversionFactor; // conversions needed. 
        armExtendPID.setSetpoint(outputRotations, ControlType.kPosition);
    }

    public void rotateArm(double angleDegrees) {
        double outputRotations = angleDegrees * -1;
        armPivotPID.setSetpoint(outputRotations, ControlType.kPosition);
    }
    
    public void pivotClaw(double angleDegrees) {
        double outputRotations = angleDegrees / 360 * 63;
        clawPivotPID.setSetpoint(outputRotations, ControlType.kPosition);
    }

    public void incrementLevel() {
        currentLevel = (currentLevel + 1) % 7;
        SmartDashboard.putString("SELECTED LEVEL", levels[currentLevel]);
    }

    public void decrementLevel() {
        currentLevel--;
        if(currentLevel < 0) {
            currentLevel = 6;
        }
        SmartDashboard.putString("SELECTED LEVEL", levels[currentLevel]);
    }

    public int getArmLevel() {
        return currentLevel;
    }

    public void setArmLevel(int level) {
        currentLevel = level;
        SmartDashboard.putString("SELECTED LEVEL", levels[currentLevel]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Primary encoder", armExtendEncoder.getPosition());
    }

    public void resetStuff() {
        armPivotPID.setSetpoint(0.1, ControlType.kDutyCycle);
    }
    public void resetClaw() {
        armExtendPID.setSetpoint(-1, ControlType.kDutyCycle);
    }

    public double getExtensionSpeed() {
        return armExtendEncoder.getVelocity();
    }

    public double getPivotSpeed() {
        return ( Math.abs(armPivotREncoder.getVelocity()) + Math.abs(armPivotLEncoder.getVelocity()) )/2;
    }

    public void start() {
        rotateArm(69);
    }

    public double getArmLevelDegrees() {
        return armPivotREncoder.getPosition();
    }

    public void pivotClawDutyCycle(double speed) {
        clawPivotPID.setSetpoint(speed, ControlType.kDutyCycle);
    }

    public void resetClawEncoderToBottom() {
        clawPivotEncoder.setPosition(ArmConstants.beginningClawPosition);
    }

    public void zeroVelocity() {
        clawPivotPID.setSetpoint(0, ControlType.kVelocity);
    }

    public void setClawPositionHere() {
        clawPivotPID.setSetpoint(clawPivotEncoder.getPosition(), ControlType.kPosition);
    }
}
