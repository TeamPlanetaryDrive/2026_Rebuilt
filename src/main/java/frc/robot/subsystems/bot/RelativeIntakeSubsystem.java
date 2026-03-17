package frc.robot.subsystems.bot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

public class RelativeIntakeSubsystem extends SubsystemBase {
    private SparkMax intakeAngleMotor;
    private SparkMax intakeSpinMotor;
    private RelativeEncoder intakeAngleMotorEncoder;
    private RelativeEncoder intakeSpinMotorEncoder;
    private SparkClosedLoopController intakeAngleMotorPID;
    private SparkClosedLoopController intakeSpinMotorPID;
    private SparkMaxConfig angleMotorConfig; 
    private SparkMaxConfig spinMotorConfig; 

    // intake motors
    // using relative encoders
    public RelativeIntakeSubsystem(){
        intakeAngleMotor = new SparkMax(Constants.intakeConstants.intakeAngleMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeSpinMotor = new SparkMax(Constants.intakeConstants.intakeSpinMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeAngleMotorEncoder = intakeAngleMotor.getEncoder();
        intakeSpinMotorEncoder = intakeSpinMotor.getEncoder();
        intakeAngleMotorPID = intakeAngleMotor.getClosedLoopController();
        intakeSpinMotorPID = intakeSpinMotor.getClosedLoopController();
        angleMotorConfig = new SparkMaxConfig();
        spinMotorConfig = new SparkMaxConfig();
        config();
    }

    public void config() {
        angleMotorConfig.closedLoop
            // slot 0 - kVelocity
            .p(0.001, ClosedLoopSlot.kSlot0)
            .i(0, ClosedLoopSlot.kSlot0)
            .d(0, ClosedLoopSlot.kSlot0)
            // slot 1 - kPosition
            .p(0.1, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1);
        
        angleMotorConfig.encoder
            .positionConversionFactor(360 / 75) // check ratio
            .velocityConversionFactor(360 / 75 / 60); // check ratio

        spinMotorConfig.closedLoop
            .p(0.001)
            .i(0)
            .d(0);

        spinMotorConfig.encoder
            .positionConversionFactor(360 / 2) // check ratio
            .velocityConversionFactor(360 / 2 / 60); // check ratio 

        angleMotorConfig.smartCurrentLimit(40);
        spinMotorConfig.smartCurrentLimit(40);

        angleMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        spinMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        intakeSpinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeAngleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeAngleMotorEncoder.setPosition(0);
        intakeSpinMotorEncoder.setPosition(0);
    }

    // set intake speed
    public void setIntakeSpeed(double radiansPerSecond){
        double rpm = 60 * radiansPerSecond / (2 * Math.PI);
        intakeSpinMotorPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setRotateSpeed(double radiansPerSecond){
        double rpm = 60 * radiansPerSecond / (2 * Math.PI);
        intakeAngleMotorPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setIntakeAngle(double degrees){
        double gearRatio = Constants.intakeConstants.intakeAngleMotorRatio; // FIX 

        // 2. Convert Degrees to Rotations
        // (Degrees / 360) gives you the fraction of a circle
        double rotations = (degrees / 360.0) * gearRatio;

        // 3. Send to the SparkMax PID Controller
        // ControlType.kPosition tells the SparkMax to go to a specific spot and stay there
        intakeAngleMotorPID.setSetpoint(rotations, SparkMax.ControlType.kPosition);
    }
    
    // start intake
    // relative intake assumes start angle is at maximum angle
    // 0 angle is the angle we need (???)
    public void start() {
        setIntakeSpeed(1000);
        setIntakeAngle(140);
    }

    public void stop() {
        intakeAngleMotorPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
        intakeSpinMotorPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
    }
}
