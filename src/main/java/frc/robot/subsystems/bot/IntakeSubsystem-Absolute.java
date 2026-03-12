package frc.robot.subsystems.bot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakePivot;
    private final SparkMax intakeSpin;
    private AbsoluteEncoder intakeAngleMotorEncoder;
    private RelativeEncoder intakeSpinMotorEncoder;
    private SparkClosedLoopController intakeAngleMotorPID;
    private SparkClosedLoopController intakeSpinMotorPID;


    // intake motors
    // using absolute encoders
    public IntakeSubsystem(){
        intakeAngleMotor = new SparkMax(Constants.intakeConstants.intakeAngleMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeSpinMotor = new SparkMax(Constants.intakeConstants.intakeSpinMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeAngleMotorEncoder = intakeAngleMotor.getAbsoluteEncoder();
        intakeSpinMotorEncoder = intakeSpinMotor.getEncoder();
        intakeAngleMotorPID = intakeAngleMotor.getClosedLoopController();
        intakeSpinMotorPID = intakeSpinMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.closedLoop.feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        
        // Apply position conversion so setpoint is in Degrees
        pivotConfig.absoluteEncoder.positionConversionFactor(360.0); 
        
        intakeAngleMotor.configure(pivotConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void configEncoders() {
        // no need to set position for absolute encoders
        // intakeAngleMotorEncoder.setPosition(0);
        intakeSpinMotorEncoder.setPosition(0);
    }

    // set intake speed
    public void setIntakeSpeed(double radiansPerSecond){
        intakeSpinMotorPID.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    // rotate intake
    public void rotateIntake(double angleDegrees){
        intakeAngleMotorPID.setSetpoint(angleDegrees, ControlType.kPosition);
    }

    // zero velocity
    public void zeroVelocity() {
        intakeAngleMotor.stopMotor();
        intakeSpinMotor.stopMotor();
    }
    
    // start intake
    // relative intake assumes start angle is at maximum angle
    // we start at some angle, and we need to rotate 140 degrees to get to the angle we need

    public void start() {
        setIntakeSpeed(1000);
        setIntakeAngle(0);
    }
    public void stop() {
        zeroVelocity();
    }
    // public IntakeSubsystem(){
    //     intakePivot = new SparkMax(Constants.intakeConstants.intakePivotCANID, SparkLowLevel.MotorType.kBrushless);
    //     intakeSpin = new SparkMax(Constants.intakeConstants.intakeSpinCANID, SparkLowLevel.MotorType.kBrushless);
    //     intakePivotEncoder = intakePivot.getAbsoluteEncoder();
    //     intakeSpinEncoder = intakeSpin.getEncoder();
    //     intakePivotPID = intakePivot.getClosedLoopController();
    //     intakeSpinPID = intakePivot.getClosedLoopController();
    // }
    // public void setIntakeSpeed(double radiansPerSecond){
    //     //do we use radians per second?
    //     intakeSpinPID.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    // }
    // public void rotateIntake(double angleDegrees){
    //     //blatant copy of the rotateArm
    //     angleDegrees *= -1;
    //     intakePivotPID.setSetpoint(angleDegrees, ControlType.kPosition);

    // }
}
