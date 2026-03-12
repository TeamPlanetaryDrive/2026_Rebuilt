package frc.robot.subsystems.bot;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooter1;
    private final SparkMax shooter2;
    private final SparkMax shooter3;
    private final SparkClosedLoopController shooter1PID;
    private final SparkClosedLoopController shooter2PID;
    private final SparkClosedLoopController shooter3PID;
    private final RelativeEncoder shooter1Encoder;
    private final RelativeEncoder shooter2Encoder;
    private final RelativeEncoder shooter3Encoder;
    private final AbsoluteEncoder shooter1Encoder;
    private final AbsoluteEncoder shooter2Encoder;
    private final AbsoluteEncoder shooter3Encoder;

    public ShooterSubsystem() {
        shooter1 = new TalonFX(Constants.shooterConstants.shooter1CANID);
        shooter2 = new TalonFX(Constants.shooterConstants.shooter2CANID);
        shooter3 = new TalonFX(Constants.shooterConstants.shooter3CANID);

        feederLeadMotor = new SparkMax(Constants.feederConstants.feeder1CANID, SparkLowLevel.MotorType.kBrushless);
        feederFollowMotor = new SparkMax(Constants.feederConstants.feeder2CANID, SparkLowLevel.MotorType.kBrushless);
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
    public void setLeaderFeederSpeed(double radiansPerSecond) {
        feederLeadMotor.setControl(m_velocityControl.withVelocity(radiansPerSecond));
        feederFollowMotor.setControl(m_velocityControl.withVelocity(radiansPerSecond));
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

    public void zeroVelocity() {
        shooter1.setControl(m_velocityControl.withVelocity(0));
        shooter2.setControl(m_velocityControl.withVelocity(0));
        shooter3.setControl(m_velocityControl.withVelocity(0));
        feederLeadMotor.setControl(m_velocityControl.withVelocity(0));
        feederFollowMotor.setControl(m_velocityControl.withVelocity(0));
    }

    public void start() {
        setAllShooterSpeeds(1000);
        setLeaderFeederSpeed(1000);
    }

    public void stop() {
        zeroVelocity();
    }




}
