package frc.robot.subsystems.bot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
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


    // intake motors
    // using relative encoders
    public RelativeIntakeSubsystem(){
        intakeAngleMotor = new SparkMax(Constants.intakeConstants.intakeAngleMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeSpinMotor = new SparkMax(Constants.intakeConstants.intakeSpinMotorCANID, SparkLowLevel.MotorType.kBrushless);
        intakeAngleMotorEncoder = intakeAngleMotor.getEncoder();
        intakeSpinMotorEncoder = intakeSpinMotor.getEncoder();
        intakeAngleMotorPID = intakeAngleMotor.getClosedLoopController();
        intakeSpinMotorPID = intakeSpinMotor.getClosedLoopController();
    }

    public void configEncoders() {
        intakeAngleMotorEncoder.setPosition(0);
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
        intakeAngleMotorPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
        intakeSpinMotorPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
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
