// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.JoystickUtil;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.Limelight;
import frc.lib.vendor.sensor.Limelight.ledMode;
import frc.lib.vendor.sensor.SendableGyro;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.AutonChooser;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;
import org.tinylog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerSwerveChassis {
  private final SendableGyro m_gyro = new ADIS16470();
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);

  private final Limelight m_limelight =
      new Limelight(
          Constants.Vision.kCameraAngleDegrees,
          Constants.Vision.kCameraHeightMeters,
          Constants.Vision.kTargetHeightMeters);

  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

  private boolean m_disableVision = false;

  private final RobotState m_state = new RobotState(m_drive, m_limelight, m_gyro, () -> 0.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainerSwerveChassis() {
    // ALL Spark Maxes should have already been initialized by this point
    SparkMax.burnFlashInSync();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    SmartDashboard.putData("Limelight", m_limelight);
    SmartDashboard.putData("JVM", new SendableJVM());

    m_limelight.setLedMode(ledMode.OFF);

    Logger.tag("Robot Container").error("This is the robot container for the swerve chassis");
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
                        JoystickUtil.squareAxis(
                            m_driveController.getLeftTriggerAxis()
                                - m_driveController.getRightTriggerAxis()),
                        !m_driveController.getLeftStickButton()),
                m_drive)
            .withName("Default Drive"));

    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
        .whenReleased(() -> m_disableVision = false);

    new JoystickButton(m_driveController, XboxController.Button.kBack.value)
        .whenReleased(() -> m_disableVision = false);
  }

  /**
   * Create mappings for use in test mode. This function runs once when this object is created.
   *
   * @return
   */
  private void configureTestModeBindings() {}

  /** This function is called each time testmode is started */
  public void testModeInit() {}

  /**
   * This function is called each loop of testmode. The scheduler is disabled in test mode, so this
   * function should run any necessary loops directly.
   */
  public void testModePeriodic() {
    m_drive.testPeriodic();
    m_state.testModePeriodic();
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
  }

  public void teleopInit() {}

  public void teleopPeriodic() {}
}
