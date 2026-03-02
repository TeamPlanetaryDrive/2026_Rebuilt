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
    SparkMax intakePivot;
    SparkMax intakeSpin;
    private AbsoluteEncoder intakePivotEncoder;
    private RelativeEncoder intakeSpinEncoder;
    private SparkClosedLoopController intakePivotPID;
    private SparkClosedLoopController intakeSpinPID;

    public IntakeSubsystem(){
        intakePivot = new SparkMax(Constants.intakeConstants.intakePivotCANID, SparkLowLevel.MotorType.kBrushless);
        intakeSpin = new SparkMax(Constants.intakeConstants.intakeSpinCANID, SparkLowLevel.MotorType.kBrushless);
        intakePivotEncoder = intakePivot.getAbsoluteEncoder();
        intakeSpinEncoder = intakeSpin.getEncoder();
        intakePivotPID = intakePivot.getClosedLoopController();
        intakeSpinPID = intakePivot.getClosedLoopController();
    }
    public void setIntakeSpeed(double radiansPerSecond){
        //do we use radians per second?
        intakeSpinEncoder.setSetPoint(radiansPerSecond, ControlType.kVelocity);
    }
    public void rotateIntake(double angleDegrees){
        //blatant copy of the rotateArm
        angleDegrees *= -1;
        intakePivotPID.setSetPoint(angleDegrees, ControlType.kPosition);

    }
}
