package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrajectoryConfig;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class ArmSubsystem {
        public static final SparkMaxConfig extendConfig = new SparkMaxConfig();
        public static final SparkMaxConfig pivotLConfig = new SparkMaxConfig();
        public static final SparkMaxConfig pivotRConfig = new SparkMaxConfig();
        public static final SparkMaxConfig clawPivotConfig = new SparkMaxConfig();
        public static final SparkMaxConfig clawSpinConfig = new SparkMaxConfig();

        static {
            extendConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
            extendConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1, 0, 0)
                .outputRange(-.75, .5);

            pivotRConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
            pivotRConfig.absoluteEncoder
                .positionConversionFactor(360);
            pivotRConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.015, 0, 0.2)
                .outputRange(-0.1, 0.1)
                .positionWrappingEnabled(true).positionWrappingInputRange(0, 360);
                
            pivotLConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .follow(Constants.ArmConstants.armPivotRCANID, true);

            clawPivotConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            clawPivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(.5, 0, 0)
                .outputRange(-.25, .25);

            clawSpinConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            clawSpinConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(.4, .0, .0)
                .outputRange(-.5, .5);
        }
    }

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            //TODO: figure out how to set feed forward without using the deprecated feedForward field
            //drivingConfig.closedLoop.outputRange(-1,1);
            // This line ^^^ was addded while I was trying to figure out how to set feed forward.
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class TrajectoryConfigs {
        public static final TrajectoryConfig config;
        static {
            config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
        }
    }
}