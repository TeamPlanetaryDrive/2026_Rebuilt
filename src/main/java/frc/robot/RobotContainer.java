// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
// jacoby is awesome
// import frc.robot.commands.arm.ArmCommand;
// import frc.robot.commands.arm.PivotUpDown;
// import frc.robot.commands.arm.ResetClaw;
// import frc.robot.commands.auto.AlignToL3;
// import frc.robot.commands.auto.OutakeClaw;
// import frc.robot.commands.auto.ToL3;
// import frc.robot.subsystems.bot.ArmSubsystem;
// import frc.robot.subsystems.bot.ClawSpinSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.subsystems.bot.ShooterSubsystem;
import frc.robot.subsystems.bot.AbsoluteIntakeSubsystem;
import frc.robot.subsystems.bot.RelativeIntakeSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.Commands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {

  // The robot's subsystems
  private final SendableChooser<Command> autoChooser;
  private final PhotonVision m_photonVision = new PhotonVision("Microsoft_LifeCam_HD-3000");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photonVision);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final RelativeIntakeSubsystem m_intake = new RelativeIntakeSubsystem();
  // private final AbsoluteIntakeSubsystem m_intake = new AbsoluteIntakeSubsystem();
  // private final ArmSubsystem m_arm = new ArmSubsystem(); 
  // private final ClawSpinSubsystem m_clawSpinner = new ClawSpinSubsystem();

  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);
  
  //private final TrajectoryFollowerBuilder follower = new TrajectoryFollowerBuilder(m_robotDrive, m_photonVision, m_robotDrive::getPose);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    //NamedCommands.registerCommand("ToL4", new ToL3(m_arm));
    // NamedCommands.registerCommand("AlignToL4", new AlignToL3(m_robotDrive, m_photonVision));
    // NamedCommands.registerCommand("OutakeClaw", new OutakeClaw(m_clawSpinner));

    configureDashboard();
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> m_robotDrive.drive(
          m_xLimiter.calculate(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)),
          m_yLimiter.calculate(-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)),
          m_rotLimiter.calculate(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
           true),
        m_robotDrive
      )
    );
  }

  // driver tab
  private void configureDashboard() {
    var driverTab = Shuffleboard.getTab("Driver");

    // UsbCamera cam = CameraServer.startAutomaticCapture();
    // cam.setResolution(1920, 1080);
    // cam.setFPS(45);
    // driverTab.add(cam);
    
// Creates the 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kRightStick.value).onTrue(new ArmCommand(m_arm)); //set arm level
    // new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(m_arm.runOnce(() -> m_arm.incrementLevel())); //increment value
    // new JoystickButton(m_driverController, Button.kRightBumper.value).onTrue(m_arm.runOnce(() -> m_arm.decrementLevel())); //decrement value
    // new JoystickButton(m_driverController, Button.kStart.value).whileTrue(m_arm.run(() -> m_arm.resetClaw())); // Climb
    
    // new JoystickButton(m_driverController, Button.kBack.value).onTrue(m_arm.runOnce(() -> m_arm.start()));
                                                                      
    // new JoystickButton(m_driverController, Button.kX.value).onTrue(m_arm.runOnce(() -> m_arm.setArmLevel(0)).andThen(new ArmCommand(m_arm))); 
    // new JoystickButton(m_driverController, Button.kA.value).onTrue(m_arm.runOnce(() -> m_arm.setArmLevel(3)).andThen(new ArmCommand(m_arm))); // Lineup to Climb
    // new JoystickButton(m_driverController, Button.kB.value).onTrue(m_arm.runOnce(() -> m_arm.setArmLevel(2)).andThen(new ArmCommand(m_arm))); // Lineup to Climb
    // new JoystickButton(m_driverController, Button.kY.value).onTrue(m_arm.runOnce(() -> m_arm.setArmLevel(1)).andThen(new ArmCommand(m_arm))); // Lineup to Climb
  
    // new POVButton(m_driverController, 90).whileTrue(new PivotUpDown(m_arm, .1));
    // new POVButton(m_driverController, 270).whileTrue(new PivotUpDown(m_arm, -.1));
    // new POVButton(m_driverController, 180).onTrue(new ResetClaw(m_arm));

    // Spin up the shooter when the Right Bumper is pressed, stop when released
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(
            Commands.sequence(
                // 1. Back up the feeder AND start the shooter wheels simultaneously
                Commands.runOnce(() -> {
                    m_shooter.feedBackward();
                    m_shooter.startShooter();
                }, m_shooter),

                // 2. Wait exactly 0.25 seconds to let the wheels spin up
                Commands.waitSeconds(0.50),

                // 3. Ram the feeder forward to fire the ball
                Commands.runOnce(() -> m_shooter.feedForward(), m_shooter)
            )
        )        // Stop everything the moment the driver lets go of the bumper
        .onFalse(m_shooter.runOnce(() -> m_shooter.stop()));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(m_intake.runOnce(() -> m_intake.start()))
        .onFalse(m_intake.runOnce(() -> m_intake.stop()));

    // Hold A to raise the feeder, release A to lower it back to 0 (FIX THIS SO IT DOES NOT GO SO FAST+GET ENCODER)
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_intake.runOnce(() -> m_intake.setRotateSpeed(-40)))  // Goes up when pressed
        .onFalse(m_intake.runOnce(() -> m_intake.coastIntakeRotate()));  // Goes down when released
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_intake.runOnce(() -> m_intake.setRotateSpeed(40)))  // Goes up when pressed
        .onFalse(m_intake.runOnce(() -> m_intake.coastIntakeRotate()));  // Goes down when released
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}