package frc.robot.subsystems.bot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClawSpinSubsystem extends SubsystemBase {

    private SparkMax clawSpin;    
    private SparkClosedLoopController clawSpinPID;    
    private RelativeEncoder clawSpinEncoder;
    
    public ClawSpinSubsystem() {
        clawSpin = new SparkMax(Constants.ArmConstants.clawSpinCANID, SparkLowLevel.MotorType.kBrushless);
        clawSpinEncoder = clawSpin.getEncoder();
        clawSpinPID = clawSpin.getClosedLoopController();
        
        clawSpin.configure(Configs.ArmSubsystem.clawSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configEncoders();
    }
    

    public void configEncoders() {
        clawSpinEncoder.setPosition(0);
    }

    public void spinClaw(double value) {
        clawSpinPID.setReference(value * Constants.ArmConstants.intakePercent, ControlType.kDutyCycle);
    }
}