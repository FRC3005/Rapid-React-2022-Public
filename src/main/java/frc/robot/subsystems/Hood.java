// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Controller;
import frc.lib.electromechanical.Encoder;
import frc.lib.util.SetpointTracking;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.tinylog.Logger;

public class Hood extends SubsystemBase implements SetpointTracking {
  private final SparkMax m_sparkMax;
  private final Controller m_controller;
  private final Encoder m_encoder;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private double m_setpoint;
  private double m_manualAdjust = 0.0;
  private DoubleSupplier m_supplier = null;
  private boolean m_hoodEnabled = false;
  private boolean m_trackingEnabled = false;
  private double m_zeroRoutineStart = 0;

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    errors += SparkMaxUtils.check(sparkMax.getEncoder().setPosition(0.0));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) Constants.Hood.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) Constants.Hood.kReverseSoftLimit));
    errors += SparkMaxUtils.check(sparkMax.setSmartCurrentLimit(20));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kPosition);
    return errors == 0;
  }

  /**
   * Set the function used for tracking with the hood.
   *
   * @param trackingSupplier function returning a double to set the hood
   */
  @Override
  public void setTrackingFunction(DoubleSupplier trackingSupplier) {
    m_supplier = trackingSupplier;
  }

  /**
   * Set a function to use as a supplier for the hood angle and start tracking if the hood is up.
   *
   * @param hoodTargetSupplier function that returns a double reperesenting the target hood angle.
   */
  @Override
  public void enableTracking() {
    if (m_supplier == null) {
      Logger.tag("Hood").warn("Tracking enabled, but no tracking function is defined!");
    } else {
      Logger.tag("Hood").info("Tracking enabled");
    }
    m_trackingEnabled = true;
  }

  /** Disable the auto tracking, restoring the setpoint last set by set(). */
  @Override
  public void disableTracking() {
    Logger.tag("Hood").info("Tracking disabled");
    m_trackingEnabled = false;
  }

  /** @return true if auto tracking is enabled. */
  public boolean isTrackingEnabled() {
    return m_trackingEnabled;
  }

  /**
   * Enable the hood. This will use either the value last set by set() or auto tracking if enabled.
   */
  public void enable() {
    m_hoodEnabled = true;
  }

  public Command enableCommand() {
    return new InstantCommand(this::enable);
  }

  /** Disable and retract the hood. */
  public void disable() {
    m_hoodEnabled = false;
  }

  public Command disableCommand() {
    return new InstantCommand(this::disable);
  }

  /** @return true if the hood is enabled. */
  public boolean isEnabled() {
    return m_hoodEnabled;
  }

  public Hood() {
    m_sparkMax = new SparkMax(Constants.Hood.kCanId).withInitializer(Hood::sparkMaxInitializer);
    m_controller = m_sparkMax.positionController(Constants.Hood.kPositionPID);

    m_encoder = m_sparkMax.builtinEncoder();
    m_encoder.setPosition(Constants.Hood.kStartingPosition);
    m_setpoint = Constants.Hood.kStartingPosition;

    m_sparkMaxSim = new CANSparkMaxSim1(m_sparkMax);
  }

  /**
   * Start rezero routine. This routine will completely stop ALL actions of the hood, and lower it
   * until it either hits the end stop, or times out.
   */
  public void rezero() {
    m_zeroRoutineStart = Timer.getFPGATimestamp();
    Logger.tag("Hood").warn("Running rehome routine starting at {}", m_zeroRoutineStart);
  }

  /**
   * Start rezero routine. This routine will completely stop ALL actions of the hood, and lower it
   * until it either hits the end stop, or times out.
   */
  public Command rezeroCommand() {
    return new InstantCommand(this::rezero);
  }

  @Override
  public void periodic() {
    // Rezero routine takes over for _any_ other functions of the hood until it is complete
    if (m_zeroRoutineStart > 0.1) {
      boolean rehomeComplete = false;
      double zeroTime = Timer.getFPGATimestamp() - m_zeroRoutineStart;
      m_sparkMax.setVoltage(Constants.Hood.kRezeroVoltage);

      // Run for a minimum time, then monitor velocity and current
      if (zeroTime >= Constants.Hood.kRezeroBlankingTimeSeconds) {
        double current = m_sparkMax.getOutputCurrent();
        double velocity = Math.abs(m_encoder.getVelocity());

        if (current > Constants.Hood.kRezeroCurrentThresholdAmps) {
          Logger.tag("Hood").warn("Rehome routine complete at {} due to current", zeroTime);
          rehomeComplete = true;
        }

        if (velocity < Constants.Hood.kRezeroVelocityThreshold) {
          Logger.tag("Hood").warn("Rehome routine complete at {} due to velocity", zeroTime);
          rehomeComplete = true;
        }

        if (zeroTime >= Constants.Hood.kRezeroTimeoutSeconds) {
          Logger.tag("Hood").warn("Rehome routine timed out.", zeroTime);
          rehomeComplete = true;
        }

        if (rehomeComplete == true) {
          m_zeroRoutineStart = 0.0;
          m_encoder.setPosition(0.0);
        }
      }

      return;
    }

    // Disabled hood stays at 0
    if (!m_hoodEnabled) {
      m_controller.setReference(0.0, m_encoder.getPosition());
      return;
    }

    double setpoint = m_setpoint;
    if (m_trackingEnabled && m_supplier != null) {
      setpoint = m_supplier.getAsDouble();
    }

    // This method will be called once per scheduler run
    m_controller.setReference(setpoint + m_manualAdjust, m_encoder.getPosition());
  }

  /**
   * Move the manual adjust by a fixed amount, the direction is set bot the sign of the input
   * argument. This can be use for something like a button to correct for a ball going long/short
   * when using vision. Manual adjustments are simply added to the setpoint. The setpoint should be
   * used for automated vision tracking or pre-programmed setpoints.
   *
   * <p>This method will not go beyond the min/max for the system, but will also not warn if it
   * attempts to, it will simply clamp.
   *
   * @param delta distance to adjust in hood units
   */
  public void bumpManualAdjust(double delta) {
    m_manualAdjust =
        MathUtil.clamp(
            m_manualAdjust + delta,
            Constants.Hood.kReverseSoftLimit,
            Constants.Hood.kForwardSoftLimit);
  }

  /**
   * Set the manual hood adjust, overwritting the previous value. This can be used to reset, or
   * reload a previous manual adjustment. Manual adjustments are simply added to the setpoint. The
   * setpoint should be used for automated vision tracking or pre-programmed setpoints.
   *
   * <p>This method will not go beyond the min/max for the system, but will also not warn if it
   * attempts to, it will simply clamp.
   *
   * @param distance distance to set the adjust in hood units
   */
  public void setManualAdjust(double distance) {
    m_manualAdjust =
        MathUtil.clamp(
            distance, Constants.Hood.kReverseSoftLimit, Constants.Hood.kForwardSoftLimit);
  }

  @Override
  public void simulationPeriodic() {
    m_sparkMaxSim.enable();
    m_sparkMaxSim.iterate(RobotController.getBatteryVoltage(), 0.02);
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    periodic();
  }

  /**
   * Set the setpoint for the hood in hood units. This will not allow setting past the min max
   * positions, and will warn if attempted to. The min and max position warning is not impacted by
   * any manual adjustments made.
   *
   * @param position setpoint for hood in hood units
   * @return true if set, false if out of range
   */
  public boolean set(double position) {
    if (isTrackingEnabled()) {
      disableTracking();
    }

    if (position < Constants.Hood.kForwardSoftLimit
        && position > Constants.Hood.kReverseSoftLimit) {
      m_setpoint = position;
      return true;
    } else {
      Logger.tag("Hood").warn("Attempting to set Hood to a bad positon {}", position);
      return false;
    }
  }

  /**
   * Get the actual position of the hood.
   *
   * @return hood position in native units (rotations of the output motor).
   */
  public double get() {
    return m_encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    addChild("Controller", m_controller);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
    builder.addBooleanProperty("Enabled", () -> m_hoodEnabled, (val) -> m_hoodEnabled = val);
    builder.addBooleanProperty("Auto Tracking Enabled", () -> isTrackingEnabled(), null);
  }
}
