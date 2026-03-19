package frc.robot.subsystems.bot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final TalonFX shooter3;
    private final SparkMax feederLeadMotor;
    private final SparkMax feederFollowMotor;
    private final RelativeEncoder feederLeadEncoder;
    private final SparkClosedLoopController feederLeadPID;
    private SparkMaxConfig feederLeadConfig;
    private SparkMaxConfig feederFollowConfig;
    private double shooterSpeed = 500;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0);

    public ShooterSubsystem() {
        this.shooter1 = new TalonFX(Constants.shooterConstants.shooter1CANID);
        this.shooter2 = new TalonFX(Constants.shooterConstants.shooter2CANID);
        this.shooter3 = new TalonFX(Constants.shooterConstants.shooter3CANID);

        configureShooterMotor(shooter1);
        configureShooterMotor(shooter2);    
        configureShooterMotor(shooter3);

        this.feederLeadMotor = new SparkMax(Constants.feederConstants.feederLeadCANID, SparkLowLevel.MotorType.kBrushless);
        this.feederFollowMotor = new SparkMax(Constants.feederConstants.feederFollowCANID, SparkLowLevel.MotorType.kBrushless);

        this.feederLeadConfig = new SparkMaxConfig();
        this.feederFollowConfig = new SparkMaxConfig();

        feederLeadConfig.closedLoop
            .velocityFF(1.0 / 5676.0)
            .p(0.0001)
            .i(0)
            .d(0);
        feederLeadConfig.encoder
            .positionConversionFactor(360.0 / (2)); // not actual ratio
            // .velocityConversionFactor(360.0 / (2) / 60); // not actual ratio 
        feederLeadConfig.smartCurrentLimit(40); //changed
        feederFollowConfig.smartCurrentLimit(40); //changed
        feederLeadConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        feederFollowConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        feederFollowConfig.follow(feederLeadMotor, true);
        feederLeadMotor.configure(feederLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederFollowMotor.configure(feederFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        this.feederLeadEncoder = feederLeadMotor.getEncoder();
        this.feederLeadPID = feederLeadMotor.getClosedLoopController();

        SmartDashboard.putNumber("Desired Shooter Speed", 500);
        SmartDashboard.putNumber("Desired Feeder Speed", 500);
    }

    // set single shooter speed
    public void setSingleShooterSpeed(TalonFX shooterPID, double radiansPerSecond) {
        // Convert Radians per Second to Rotations per Second for Phoenix 6
        double rotationsPerSecond = radiansPerSecond / (2 * Math.PI);

        shooterPID.setControl(m_velocityControl.withVelocity(rotationsPerSecond));

    }

    // all shooter speeds are the same
    public void setAllShooterSpeeds(double radiansPerSecond) {
        setSingleShooterSpeed(shooter1, radiansPerSecond);
        setSingleShooterSpeed(shooter2, radiansPerSecond);
        setSingleShooterSpeed(shooter3, radiansPerSecond);
    }

    // set single feeder speed
    // follower follows leader motor speed
    public void setFeederSpeed(double radiansPerSecond) {
        double rotationsPerSecond = radiansPerSecond / (2 * Math.PI);
        double targetRPM = rotationsPerSecond * 60;

        // CHANGED: Use setReference and assign it to Slot 0
        feederLeadPID.setReference(targetRPM, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    // get single shooter speed
    public double getSingleShooterSpeed(TalonFX shooterPID) {
        return shooterPID.getControlVoltage();
    }

    // get leader feeder speed
    public double getLeaderFeederSpeed(SparkMax feederPID) {
        return feederPID.getControlVoltage();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter1 Speed", getSingleShooterSpeed(shooter1));
        SmartDashboard.putNumber("Shooter2 Speed", getSingleShooterSpeed(shooter2));
        SmartDashboard.putNumber("Shooter3 Speed", getSingleShooterSpeed(shooter3));
        SmartDashboard.putNumber("Leader Feeder Speed", getLeaderFeederSpeed(feederLeadMotor));
        SmartDashboard.putNumber("Follower Feeder Speed", getFollowerFeederSpeed(feederFollowMotor));
    }

    public void stop() {
        shooter1.setControl(new CoastOut());
        shooter2.setControl(new CoastOut());
        shooter3.setControl(new CoastOut());
        feederLeadMotor.set(0);
    }

    // 1. Spin up just the shooter wheels
    public void startShooter() {
        // max ~ 614
        setAllShooterSpeeds(SmartDashboard.getNumber("Desired Shooter Speed", 500)); 
    }

    // 2. Run the feeder forward to shoot
    public void feedForward() {
        setFeederSpeed(SmartDashboard.getNumber("Desired Feeder Speed", 500)); 
    }

    // 3. Run the feeder backward to un-jam / get a running start
    public void feedBackward() {
        setFeederSpeed(-100); // Negative speed to pull the ball away from the wheels
    }
}
