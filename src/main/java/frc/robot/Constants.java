package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.path.PathConstraints;

public final class Constants {
    public static final class shooterConstants {
        public static final int shooter1CANID = 20; 
        public static final int shooter2CANID = 21; 
        public static final int shooter3CANID = 22; 
    }

    public static final class feederConstants {
        public static final int feederLeadCANID = 2; // Make sure they are correct order
        public static final int feederFollowCANID = 18; 
    }

    public static final class intakeConstants {
        public static final int intakeAngleMotorCANID = 3; 
        public static final int intakeSpinMotorCANID = 5;

        public static final double intakeAngleMotorRatio = 1; // Changed from 0

    }

    public static final class PhotonVisionConstants {

        public static final int[] redCoralReefIds = {6, 7, 8, 9, 10, 11}; 
        public static final int[] blueCoralReefIds = {17, 18, 19, 20, 21, 22}; 

        public static final double offsetXMeters = 0.30226;
        public static final double offsetYMeters = 0.3429;
        public static final double offsetZMeters = .2286;
        public static final double rotationXMeters = 0;
        public static final double rotationYMeters = 0;
        public static final double rotationZMeters = 0;
        public static final Transform3d transform = new Transform3d(new Translation3d(offsetXMeters, offsetYMeters, offsetZMeters),
                                                                    new Rotation3d(rotationXMeters, rotationYMeters, rotationZMeters));
    }
    
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            // Front right
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            // Back left
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            // Back right
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 16;
        public static final int kRearLeftDrivingCanId = 15;
        public static final int kFrontRightDrivingCanId = 1;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 12;
        public static final int kRearLeftTurningCanId = 11;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 13;

        public static final boolean kGyroReversed = true;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kDistanceTolerance = 0.1;
        public static final double kLostTagCancelSec = 1.0;
        public static final int kTargetTagId = 26; //change
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.1;
        public static final String cameraName = "camera1";
    }

    public static final class VisionConstants {
        //camera constants
        public static final Transform3d cameraPositionRobotRel = new Transform3d(new Translation3d(-0.5, 0, -0.5), new Rotation3d());
        public static final Transform3d robotPositionCameraRel = cameraPositionRobotRel.inverse();
        public static final int photonVisionPort = 2856;
        public static final String s_photonVisionPort = "5800";
    }

    public static final class AutoConstants {
        // acceleration and angular per second
        public static final double kMoveBackDistance = 2.5; //meters
        public static final double kMaxSpeedMetersPerSecond = 1.2; //updated from 1.0
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        //controllers
        public static final double kPXController = 0.0004;
        public static final double kPYController = 0.0002;
        public static final double kPThetaController = 0.00001;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    //Neo Motor Constants
    public static final class NeoMotorConstants {
        // the fact this gets its own class pisses me off
        public static final double kFreeSpeedRpm = 5676;
    }

}
