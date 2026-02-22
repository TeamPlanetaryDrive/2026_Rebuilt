package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    /* TECHNICALLY, these all need getters, but that would genuinely just
    overcomplicate things, because getClimbAngle() is just longer than climbAngle.
    This is also technically a data leak/lack of encapsulation, but again, I think
    having getters for everything would be over the top and decrease readibility.
    */
    public static final class ArmConstants {
        public static final double beginningArmPosition = -23;
        public static final double beginningClawPosition = -15;

        public static final int armExtendCANID = 1; 
        public static final int armPivotRCANID = 2; 
        public static final int armPivotLCANID = 3; 
        public static final int clawPivotCANID = 4;
        public static final int clawSpinCANID = 5; 

        // rotations to cm
        public static final double armExtensionConversionFactor = 5.793103448;

        public static final double intakeAngle = 52;  // deg
        public static final double l1Angle = 45; 
        public static final double l2Angle = 55; 
        public static final double l3Angle = 72; 
        public static final double l4Angle = 72; 
        public static final double climbAngle = 80;

        public static final double intakeExtension = 1.25;  // in
        public static final double l1Extension = 4; 
        public static final double l2Extension = 12; 
        public static final double l3Extension = 14; 
        public static final double l4Extension = 43.5; 
        public static final double climbExtension = 0;

        public static final double intakeClawAngle = -15;
        public static final double l1ClawAngle = -45;
        public static final double l2ClawAngle = -65;
        public static final double l3ClawAngle = -85;
        public static final double l4ClawAngle = -85;
        public static final double climbClawAngle = 0;

        public static final double intakePercent = .8;
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
        public static final int kFrontLeftDrivingCanId = 18;
        public static final int kRearLeftDrivingCanId = 15;
        public static final int kFrontRightDrivingCanId = 17;
        public static final int kRearRightDrivingCanId = 16;

        public static final int kFrontLeftTurningCanId = 14;
        public static final int kRearLeftTurningCanId = 11;
        public static final int kFrontRightTurningCanId = 13;
        public static final int kRearRightTurningCanId = 12;

        public static final boolean kGyroReversed = true;
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
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        //controllers
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

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
