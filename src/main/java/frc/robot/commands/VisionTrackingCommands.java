// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.vendor.sensor.Limelight;
import frc.lib.vendor.sensor.Limelight.ledMode;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;

public class VisionTrackingCommands {
  private final Hood m_hood;
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Limelight m_limelight;
  private final LinearFilter m_lfilter;
  private final Supplier<Pose2d> m_turretPoseSupplier;

  private double m_lastHoodPosition = 0.0;
  private double m_lastRpm = 0.0;
  private double m_lastTurretPosition = 0.0;
  private boolean m_usePoseToTarget = false;
  private double m_lastTargetTimestamp = 0.0;

  public VisionTrackingCommands(
      Turret turret,
      Hood hood,
      Shooter shooter,
      Limelight limelight,
      Supplier<Pose2d> turretPoseSupplier) {
    m_hood = hood;
    m_turret = turret;
    m_shooter = shooter;
    m_limelight = limelight;
    m_turretPoseSupplier = turretPoseSupplier;

    m_lfilter = LinearFilter.movingAverage(Constants.Vision.kTurretMovingAverageSampleLength);

    m_limelight.setLimelightValues(
        Limelight.ledMode.ON, Limelight.camMode.VISION_PROCESSING, Limelight.pipeline.PIPELINE0);

    SmartDashboard.putNumber("Turret Vision Gain", Constants.Vision.kTurretGain);
    m_lastTargetTimestamp = Timer.getFPGATimestamp();
  }

  private double getTargetDistance() {
    // return RobotState.centerOfGoalFromVisionTranslation(m_limelight.getTargetTranslation())
    //    .getNorm();

    return m_limelight.getTargetTranslation().getNorm();
  }

  private boolean isTargetActive() {
    if (m_limelight.isValidTarget()) {
      m_lastTargetTimestamp = Timer.getFPGATimestamp();
      return true;
    }
    double timeDelta = Timer.getFPGATimestamp() - m_lastTargetTimestamp;
    SmartDashboard.putNumber("Time Delta", timeDelta);
    return timeDelta < Constants.Vision.kTargetTimeoutSeconds;
  }

  private double rpmTrackingFunction() {
    if (!m_shooter.isShooting()) {
      if (m_usePoseToTarget) {
        m_lastRpm =
            Constants.Vision.kRpmLookup.get(
                RobotState.getPoseToGoal(m_turretPoseSupplier.get()).getTranslation().getNorm());
      } else if (m_limelight.isValidTarget()) {
        m_lastRpm = Constants.Vision.kRpmLookup.get(getTargetDistance());
      }
    }
    return m_lastRpm;
  }

  private double hoodTrackingFunction() {
    if (!m_shooter.isShooting()) {
      if (m_usePoseToTarget) {
        m_lastHoodPosition =
            Constants.Vision.kHoodLookup.get(
                RobotState.getPoseToGoal(m_turretPoseSupplier.get()).getTranslation().getNorm());
      } else if (m_limelight.isValidTarget()) {
        m_lastHoodPosition = Constants.Vision.kHoodLookup.get(getTargetDistance());
      }
    }
    return m_lastHoodPosition;
  }

  private double calcTargetAngle() {
    double turretGain =
        SmartDashboard.getNumber("Turret Vision Gain", Constants.Vision.kTurretGain);
    return m_turret.getDegrees() + turretGain * m_lfilter.calculate(-m_limelight.getOffSetX());
  }

  private double turretTrackingFunction() {
    if (m_usePoseToTarget) {
      return RobotState.getPoseToGoal(m_turretPoseSupplier.get()).getRotation().getDegrees()
          + m_turret.getDegrees();
    }

    if (m_limelight.isValidTarget() && !m_shooter.isShooting()) {
      m_lastTurretPosition = calcTargetAngle() + Constants.Vision.kLimelightXOffset;
    }

    if (!isTargetActive()) {
      m_lastTurretPosition = 0.0;
    }

    return m_lastTurretPosition;
  }

  /**
   * Change whether to use generic camera Tx/Ty angles, or estimated robot pose for targeting. The
   * estimated pose is based on the pose supplier value.
   *
   * @param enable set true to use estimated robot pose
   */
  public void usePoseToTarget(boolean enable) {
    m_usePoseToTarget = enable;
  }

  /**
   * Set the tracking functions to the ones in this file. Typically called as an initialization
   * routine.
   */
  public void setupTrackingFunctions() {
    m_hood.setTrackingFunction(this::hoodTrackingFunction);
    m_shooter.setTrackingFunction(this::rpmTrackingFunction);
    m_turret.setTrackingFunction(this::turretTrackingFunction);
  }

  /**
   * Enable tracking. This only enables the tracking functions and does not turn on the limelight
   *
   * @return
   */
  public Command enableTracking() {
    return new ParallelCommandGroup(
            m_hood.enableTrackingCommand(),
            m_shooter.enableTrackingCommand(),
            m_turret.enableTrackingCommand(),
            turnOnLed())
        .withName("Enable Tracking");
  }

  /**
   * Disable tracking. This only disables the tracking functions and does not set the limelight or
   * other functions
   *
   * @return new command to disable tracking
   */
  public Command disableTracking() {
    return new ParallelCommandGroup(
            m_hood.disableTrackingCommand(),
            m_shooter.disableTrackingCommand(),
            m_turret.disableTrackingCommand(),
            turnOffLed())
        .withName("Disable Tracking");
  }

  /**
   * Turn on the vision LED
   *
   * @return new command to turn on the LED
   */
  public Command turnOnLed() {
    return new InstantCommand(() -> m_limelight.setLedMode(ledMode.ON)).withName("Turn on LED");
  }

  /**
   * Turn off the vision LED
   *
   * @return new command to turn off the LED
   */
  public Command turnOffLed() {
    return new InstantCommand(() -> m_limelight.setLedMode(ledMode.OFF)).withName("Turn off LED");
  }

  /**
   * Blink the LED using LEDMode.Blink of the limelight, then turn off the LED
   *
   * @param durationSeconds seconds to blink for
   * @return new command to blink the LED
   */
  public Command blinkLed(double durationSeconds) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> m_limelight.setLedMode(ledMode.BLINK)),
            new WaitCommand(durationSeconds),
            turnOffLed())
        .withName("Blink LED");
  }

  public boolean onTarget() {
    // TODO: Implement
    return m_limelight.isValidTarget();
  }

  public Command waitOnTarget(double timeoutSeconds) {
    return new WaitUntilCommand(this::onTarget)
        .withTimeout(timeoutSeconds)
        .withName("Wait on target " + timeoutSeconds + " seconds");
  }
}
