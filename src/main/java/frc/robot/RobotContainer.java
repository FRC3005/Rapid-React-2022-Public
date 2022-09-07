// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.JoystickUtil;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.ADIS16470.ADIS16470CalibrationTime;
import frc.lib.vendor.sensor.Limelight;
import frc.lib.vendor.sensor.Limelight.ledMode;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.ComplexRight;
import frc.robot.auton.FourBallCenter;
import frc.robot.auton.SimpleDriveBackAndShoot;
import frc.robot.auton.TwoPlusTwoLeft;
import frc.robot.auton.locations.Location;
import frc.robot.commands.VisionTrackingCommands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import org.tinylog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ADIS16470 m_gyro =
      new ADIS16470(
          Constants.kCompetitionMode ? ADIS16470CalibrationTime._8s : ADIS16470CalibrationTime._1s);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);

  private final Indexer m_indexer = new Indexer();

  private final Turret m_turret = new Turret();

  private final Intake m_intake = new Intake();

  private final Hood m_hood = new Hood();

  private final Climber m_climber = new Climber();

  private final Shooter m_shooter = new Shooter(m_indexer);

  private final Limelight m_limelight =
      new Limelight(
          Constants.Vision.kCameraAngleDegrees,
          Constants.Vision.kCameraHeightMeters,
          Constants.Vision.kTargetHeightMeters);

  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final RobotState m_robotState =
      new RobotState(m_drive, m_limelight, m_gyro, () -> m_turret.getDegrees() + 180);

  private final VisionTrackingCommands m_visionCommands =
      new VisionTrackingCommands(
          m_turret,
          m_hood,
          m_shooter,
          m_limelight,
          () -> m_robotState.getChassisToTurret(m_robotState.getPoseEstimate()));

  /* List Autons here */
  {
    // new DriveBackAndShootRight(m_drive, m_shooter, m_indexer, m_intake, m_hood);
    // new SwerveRotateTuning(m_drive);
    // new SwerveDriveTuning(m_drive);
    // new SwerveDriveBackTuning(m_drive);
    // new SwerveCombinedTuning(m_drive);
    new ComplexRight(
        m_drive, m_shooter, m_indexer, m_intake, m_hood, m_turret, m_visionCommands, 3);
    new ComplexRight(
        m_drive, m_shooter, m_indexer, m_intake, m_hood, m_turret, m_visionCommands, 5);
    new SimpleDriveBackAndShoot(
        m_drive,
        m_shooter,
        m_indexer,
        m_intake,
        m_hood,
        m_turret,
        m_climber,
        m_visionCommands,
        Location.RightStart());
    new SimpleDriveBackAndShoot(
        m_drive,
        m_shooter,
        m_indexer,
        m_intake,
        m_hood,
        m_turret,
        m_climber,
        m_visionCommands,
        Location.CenterStart());
    new SimpleDriveBackAndShoot(
        m_drive,
        m_shooter,
        m_indexer,
        m_intake,
        m_hood,
        m_turret,
        m_climber,
        m_visionCommands,
        Location.LeftStart());
    new FourBallCenter(m_drive, m_shooter, m_indexer, m_intake, m_hood, m_turret, m_visionCommands);
    new TwoPlusTwoLeft(m_drive, m_shooter, m_indexer, m_intake, m_hood, m_turret, m_visionCommands);
    AutonChooser.setDefaultAuton(
        new SimpleDriveBackAndShoot(
            m_drive,
            m_shooter,
            m_indexer,
            m_intake,
            m_hood,
            m_turret,
            m_climber,
            m_visionCommands,
            Location.CenterStart()));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // ALL Spark Maxes should have already been initialized by this point
    SparkMax.burnFlashInSync();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    SmartDashboard.putData("Limelight", m_limelight);
    SmartDashboard.putData("JVM", new SendableJVM());

    m_limelight.setLedMode(ledMode.ON);

    // Set up tracking functions for all modes
    m_visionCommands.setupTrackingFunctions();
  }

  private void configureButtonBindings() {
    /*************************************************
     * Driver controls
     *************************************************/
    m_drive.setDefaultCommand(
        new RunCommand(
                () ->
                    m_drive.drive(
                        MathUtil.applyDeadband(-m_driveController.getLeftY(), 0.1),
                        MathUtil.applyDeadband(-m_driveController.getLeftX(), 0.1),
                        /*m_driveController.getLeftTriggerAxis()
                        - m_driveController.getRightTriggerAxis(),*/
                        JoystickUtil.squareAxis(
                            m_driveController.getLeftTriggerAxis()
                                - m_driveController.getRightTriggerAxis()),
                        !m_driveController.getLeftStickButton()),
                m_drive)
            .withName("Default Drive"));

    new JoystickButton(m_driveController, XboxController.Button.kA.value)
        .whenReleased(() -> {
            
                m_indexer.intakeCommand().raceWith(m_intake.intakeOutCommand()).schedule();
            
        });

    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .whenReleased(() -> {
            
                m_indexer.intakeCancelCommand().alongWith(m_intake.intakeInCommand()).schedule();
            
        });

    new JoystickButton(m_driveController, XboxController.Button.kX.value)
        .whileHeld(
            m_intake.intakeOutReversedCommand().alongWith(m_indexer.reverseIndexerCommand()));

    new JoystickButton(m_driveController, XboxController.Button.kRightStick.value)
        .whenReleased(() -> m_drive.zeroHeading());

    new JoystickButton(m_driveController, XboxController.Button.kY.value)
        .whenReleased(m_shooter.idleCommand());

    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
        .whenReleased(
            () -> {
              disableAllTracking();
              m_turret.setDegrees(0.0);
            });

    new JoystickButton(m_driveController, XboxController.Button.kBack.value)
        .whenReleased(
            () -> {
              enableAllTracking();
              m_turret.setDegrees(0.0);
            });

    /*************************************************
     * Combined controls
     *************************************************/
    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .or(new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value))
        .whenActive(
            new InstantCommand(
                    () -> {
                      m_driveController.setRumble(
                          RumbleType.kRightRumble, Constants.OIConstants.kShootRumbleStrength);
                      m_operatorController.setRumble(
                          RumbleType.kRightRumble, Constants.OIConstants.kShootRumbleStrength);
                    })
                .andThen(m_indexer.adjustCommand())
                .andThen(m_shooter.spinUpCommand())
                .withInterrupt(
                    () ->
                        !m_driveController.getRightBumper()
                            && !m_operatorController.getRightBumper()))
        .whenInactive(
            new InstantCommand(
                () -> {
                  m_driveController.setRumble(RumbleType.kRightRumble, 0.0);
                  m_operatorController.setRumble(RumbleType.kRightRumble, 0.0);
                }));

    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .and(new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value))
        .whenActive(m_shooter.shootCommand(), true);

    /*************************************************
     * Operator controls
     *************************************************/
    m_climber.setDefaultCommand(
        new RunCommand(
            () -> {
              // m_climber.swordManual(JoystickUtil.squareAxis(-m_operatorController.getLeftY()));
              m_climber.elevatorManual(JoystickUtil.squareAxis(-m_operatorController.getRightY()));
            },
            m_climber));

    new Trigger(() -> m_climber.isClimbing() && m_turret.isTrackingEnabled())
        .whenActive(
            new InstantCommand(() -> disableAllTracking()).alongWith(m_intake.intakeInCommand()));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whenActive(() -> m_climber.resetSpears());

    new Trigger(() -> m_operatorController.getStartButton() && m_operatorController.getBackButton())
        .whenActive(
            m_climber
                .prepareClimber()
                .withTimeout(4.0)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          disableAllTracking();
                          m_turret.setDegrees(0.0);
                        })));

    new Trigger(() -> m_operatorController.getPOV() == 0) // UP on the operator d-pad
        .whenActive(
            () -> {
              disableAllTracking();
              m_turret.setDegrees(0.0);
              m_shooter.setShotRpm(Constants.ShotPresets.kShot1RPM);
              m_hood.set(Constants.ShotPresets.kShot1Hood);
            },
            m_hood,
            m_shooter);

    new Trigger(
            () ->
                m_operatorController.getPOV() == 90
                    || m_operatorController.getPOV() == 270) // LEFT or RIGHT on the operator d-pad
        .whenActive(
            () -> {
              disableAllTracking();
              m_turret.setDegrees(0.0);
              m_shooter.setShotRpm(Constants.ShotPresets.kShot2RPM);
              m_hood.set(Constants.ShotPresets.kShot2Hood);
            },
            m_hood,
            m_shooter);

    new Trigger(() -> m_operatorController.getPOV() == 180) // down on the operator d-pad
        .whenActive(
            () -> {
              disableAllTracking();
              m_turret.setDegrees(0.0);
              m_shooter.setShotRpm(Constants.ShotPresets.kShot3RPM);
              m_hood.set(Constants.ShotPresets.kShot3Hood);
            },
            m_hood,
            m_shooter);
  }

  /**
   * Create mappings for use in test mode. This function runs once when this object is created.
   *
   * @return
   */
  private void configureTestModeBindings() {}

  /** This function is called each time testmode is started */
  public void testModeInit() {
    m_hood.disableTrackingCommand();
    m_shooter.disableTrackingCommand();
    m_turret.disableTrackingCommand();
    m_limelight.setLedMode(ledMode.OFF);
  }

  /**
   * This function is called each loop of testmode. The scheduler is disabled in test mode, so this
   * function should run any necessary loops directly.
   */
  public void testModePeriodic() {
    m_drive.testPeriodic();
    m_shooter.testModePeriodic();
    m_indexer.testModePeriodic();
    m_turret.testModePeriodic();
    m_intake.testModePeriodic();
    m_hood.testModePeriodic();
    m_robotState.testModePeriodic();
    m_climber.testModePeriodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Logger.tag("Auton").info("Auton selected: {}", AutonChooser.getAuton().getName());

    return AutonChooser.getAuton();
  }

  /** Run a function at the start of auton. */
  public void autonInit() {
    m_drive.calibrateGyro();
    m_drive.stop();

    // Set the climber to be ready for the match
    /*
      m_climber
          .linearToInPosCommand()
          .andThen(m_climber.pivotToDrivePosCommand())
          .withName("Climber Auton Start")
          .schedule();
    */
  }

  private void enableAllTracking() {
    m_hood.enableTracking();
    m_turret.enableTracking();
    m_shooter.enableTracking();
    m_limelight.setLedMode(ledMode.ON);
  }

  private void disableAllTracking() {
    m_hood.disableTracking();
    m_turret.disableTracking();
    m_shooter.disableTracking();
    m_limelight.setLedMode(ledMode.OFF);
  }

  public void teleopInit() {
    m_visionCommands.usePoseToTarget(false);

    m_hood.enable();
    enableAllTracking();
  }

  public void disabledInit() {
    m_limelight.setLedMode(ledMode.ON);
  }
}
