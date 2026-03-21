package frc.robot.subsystems.bot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            .d(0, ClosedLoopSlot.kSlot1)
            // make the motor less violent
            .outputRange(-0.3, 0.3, ClosedLoopSlot.kSlot1);
        
        angleMotorConfig.encoder
            .positionConversionFactor(360.0 / 75.0) // check ratio
            .velocityConversionFactor(360.0 / 75.0 / 60.0); // check ratio

        spinMotorConfig.closedLoop
            .p(0.001)
            .i(0)
            .d(0);

        spinMotorConfig.encoder
            .positionConversionFactor(360.0 / 2.0) // check ratio
            .velocityConversionFactor(360.0 / 2.0 / 600); // check ratio 

        angleMotorConfig.smartCurrentLimit(40);
        spinMotorConfig.smartCurrentLimit(40);

        angleMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        spinMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        // Making moters move the correct way:
        spinMotorConfig.inverted(true);
        angleMotorConfig.inverted(true);

        intakeSpinMotor.configure(spinMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeAngleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeAngleMotorEncoder.setPosition(0);
        intakeSpinMotorEncoder.setPosition(0);
    }

    // set intake speed
    public void setIntakeSpeed(double radiansPerSecond){
        double rpm = 60 * radiansPerSecond / (2 * Math.PI);
        // CHANGED: Use setReference and force Slot 0
        intakeSpinMotorPID.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setRotateSpeed(double radiansPerSecond, int currentLimit){
        double rpm = 60 * radiansPerSecond / (2 * Math.PI);
        angleMotorConfig.smartCurrentLimit(currentLimit);
        intakeAngleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // CHANGED: Use setReference and force Slot 0
        intakeAngleMotorPID.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setIntakeAngle(double degrees){
        // CHANGED: Removed the extra math, and forced it to use Slot 1
        intakeAngleMotorPID.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public void coastIntakeRotate(){
        intakeAngleMotor.set(0);
        intakeSpinMotor.set(0);
    }
    
    // start intake
    public void start() {
        setIntakeSpeed(600);
        setRotateSpeed(-4, 10);
        // CHANGED: Removed setIntakeAngle so this method ONLY spins the rollers
    }

    public void rotateAndSpin(){
        setIntakeSpeed(600);
        setRotateSpeed(-40,40);

    }

    public void stop() {
        // CHANGED: Use setReference and force Slot 0
        intakeSpinMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Angle Temp C", intakeAngleMotor.getMotorTemperature());
        SmartDashboard.putNumber("Intake Spin Temp C", intakeSpinMotor.getMotorTemperature());
        
        // ADDED: Lets you see exactly what angle the arm thinks it is at!
        SmartDashboard.putNumber("Current Intake Angle", intakeAngleMotorEncoder.getPosition());
    }
}