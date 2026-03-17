package frc.robot.subsystems.bot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class AbsoluteIntakeSubsystem extends SubsystemBase {
    private SparkMax intakeAngleMotor;
    private SparkMax intakeSpinMotor;
    private AbsoluteEncoder intakeAngleMotorEncoder;
    private RelativeEncoder intakeSpinMotorEncoder;
    private SparkClosedLoopController intakeAngleMotorPID;

    // Intake motors
    public AbsoluteIntakeSubsystem(){
        intakeAngleMotor = new SparkMax(Constants.intakeConstants.intakeAngleMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeSpinMotor = new SparkMax(Constants.intakeConstants.intakeSpinMotorCANID, SparkLowLevel.MotorType.kBrushless);
        
        intakeAngleMotorEncoder = intakeAngleMotor.getAbsoluteEncoder();
        intakeSpinMotorEncoder = intakeSpinMotor.getEncoder();
        
        intakeAngleMotorPID = intakeAngleMotor.getClosedLoopController();

        configEncoders();
    }

    public void configEncoders() {
        // 1. Configure the Angle Motor (Requires precise position PID)
        com.revrobotics.spark.config.SparkMaxConfig angleConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        
        angleConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
            .p(0.01) // Set your PID gains here
            .i(0.0)
            .d(0.0);

        intakeAngleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 2. Configure the Spin Motor (Simple percent output, resets old PID memory to be safe)
        com.revrobotics.spark.config.SparkMaxConfig spinConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        intakeSpinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Set intake speed using percent power (e.g., 0.5 = 50% speed, -0.5 = 50% reverse)
    public void setIntakeSpeed(double percentPower){
        intakeSpinMotor.set(percentPower);
    }

    // Rotate intake (Direct PID setpoint without ratio math)
    public void rotateIntake(double angleDegrees){
        intakeAngleMotorPID.setSetpoint(angleDegrees, ControlType.kPosition);
    }

    // Safely cut power to all motors in this subsystem
    public void zeroVelocity() {
        intakeAngleMotor.stopMotor();
        intakeSpinMotor.stopMotor();
    }

    // Mathematically calculate and set intake angle based on gear ratio
    public void setIntakeAngle(double degrees){
        double gearRatio = Constants.intakeConstants.intakeAngleMotorRatio; 

        // Convert Degrees to Rotations
        double rotations = (degrees / 360.0) * gearRatio;

        // Send to the SparkMax PID Controller
        intakeAngleMotorPID.setSetpoint(rotations, SparkMax.ControlType.kPosition);
    }
    
    // Start intake sequence
    public void start() {
        setIntakeSpeed(0.1); // Spins wheels at 10% power
        setIntakeAngle(140); // Pivots arm to 140 degrees
    }
    
    // Stop intake sequence
    public void stop() {
        zeroVelocity();
    }
}