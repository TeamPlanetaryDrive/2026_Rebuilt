package frc.robot.subsystems.bot;

import java.util.HashMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax armExtend;
    private SparkMax armPivotL; 
    private SparkMax armPivotR;
    private SparkMax clawPivot;

    private SparkClosedLoopController armExtendPID; 
    private SparkClosedLoopController armPivotPID; 
    private SparkClosedLoopController clawPivotPID;

    private RelativeEncoder armExtendEncoder; 
    private RelativeEncoder armPivotLEncoder; 
    private AbsoluteEncoder armPivotREncoder;
    private RelativeEncoder clawPivotEncoder;

    private int currentLevel = 0;
    private HashMap<Integer, String> map;

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
        map = new HashMap<>();
        map.put(0, "INTAKE");
        map.put(1, "L1");
        map.put(2, "L2");
        map.put(3, "L3");
        map.put(4, "L4");
        map.put(5, "CLIMB");
        map.put(6, "BOTTOM");
        map.put(7, "VERTICAL");

        SmartDashboard.putString("SELECTED LEVEL", map.get(currentLevel));
    }

    public void configEncoders() {
        armExtendEncoder.setPosition(0);
        clawPivotEncoder.setPosition(Constants.ArmConstants.beginningClawPosition);
    }
    
    public void extendArm(double distance) {
        double outputRotations = distance * Constants.ArmConstants.armExtensionConversionFactor; // conversions needed. 
        armExtendPID.setReference(outputRotations, ControlType.kPosition);
    }

    public void rotateArm(double angleDegrees) {
        double outputRotations = angleDegrees * -1;
        armPivotPID.setReference(outputRotations, ControlType.kPosition);
    }
    
    public void pivotClaw(double angleDegrees) {
        double outputRotations = angleDegrees / 360 * 63;
        clawPivotPID.setReference(outputRotations, ControlType.kPosition);
    }

    public void incrementLevel() {
        currentLevel = (currentLevel + 1) % 7;
        SmartDashboard.putString("SELECTED LEVEL", map.get(currentLevel));
    }

    public void decrementLevel() {
        currentLevel--;
        if(currentLevel < 0) {
            currentLevel = 6;
        }
        SmartDashboard.putString("SELECTED LEVEL", map.get(currentLevel));
    }

    public int getArmLevel() {
        return currentLevel;
    }

    public void setArmLevel(int level) {
        currentLevel = level;
        SmartDashboard.putString("SELECTED LEVEL", map.get(currentLevel));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Primary encoder", armExtendEncoder.getPosition());
    }

    public void resetStuff() {
        armPivotPID.setReference(0.1, ControlType.kDutyCycle);
    }
    public void resetClaw() {
        armExtendPID.setReference(-1, ControlType.kDutyCycle);
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
        clawPivotPID.setReference(speed, ControlType.kDutyCycle);
    }

    public void resetClawEncoderToBottom() {
        clawPivotEncoder.setPosition(ArmConstants.beginningClawPosition);
    }

    public void zeroVelocity() {
        clawPivotPID.setReference(0, ControlType.kVelocity);
    }

    public void setClawPositionHere() {
        clawPivotPID.setReference(clawPivotEncoder.getPosition(), ControlType.kPosition);
    }
}
