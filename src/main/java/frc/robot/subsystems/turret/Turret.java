// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Controller;
import frc.lib.controller.ProfiledPositionController;
import frc.lib.electromechanical.Encoder;
import frc.lib.util.SetpointTracking;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.tinylog.Logger;

public class Turret extends SubsystemBase implements SetpointTracking {
  private final SparkMax m_sparkMax;
  private final Controller m_controller;
  private final Encoder m_encoder;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private final SimpleMotorFeedforward m_feedforward;
  private double m_setpoint;

  private boolean m_trackingEnabled = false;
  private DoubleSupplier m_supplier = null;

  // Test Mode
  private boolean m_testModeEnabled = false;

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) Constants.Turret.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) Constants.Turret.kReverseSoftLimit));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                sparkMax.getEncoder(), Constants.Turret.kGearRatio));
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    errors += SparkMaxUtils.check(sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true));
    errors += SparkMaxUtils.check(sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocityAndPosition);
    return errors == 0;
  }

  public Turret() {
    m_sparkMax = new SparkMax(Constants.Turret.kCanId).withInitializer(Turret::sparkMaxInitializer);
    m_controller =
        new ProfiledPositionController(
            m_sparkMax.positionController(Constants.Turret.kPositionPID),
            Constants.Turret.kProfileConstraints);

    m_encoder = m_sparkMax.builtinEncoder();
    m_encoder.setPosition(Constants.Turret.kStartingPosition);
    m_setpoint = Constants.Turret.kStartingPosition;
    m_feedforward = new SimpleMotorFeedforward(Constants.Turret.kS, Constants.Turret.kV);
    m_sparkMaxSim =
        new CANSparkMaxSim1(
            m_sparkMax,
            new FlywheelSim(
                DCMotor.getNeo550(1), Constants.Turret.kGearRatio, Constants.Turret.kInteria));
  }

  /**
   * Get the position of the turret
   *
   * @return Position of the turret in degrees
   */
  public double getDegrees() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    double setpoint = m_setpoint;
    if (m_trackingEnabled && m_supplier != null) {
      setpoint = m_supplier.getAsDouble();
    }

    m_controller.setReference(
        setpoint, m_encoder.getPosition(), m_feedforward.calculate(m_encoder.getVelocity()));
  }

  @Override
  public void simulationPeriodic() {
    m_sparkMaxSim.iterate(RobotController.getBatteryVoltage(), 0.02);
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (m_testModeEnabled) {
      periodic();
    }
  }

  public boolean setDegrees(double degrees) {
    if (degrees < Constants.Turret.kForwardSoftLimit
        && degrees > Constants.Turret.kReverseSoftLimit) {
      m_setpoint = degrees;
      return true;
    } else {
      Logger.tag("Turret").warn("Attempting to set turret to a bad angle {}", degrees);
      return false;
    }
  }

  public Command setDegreesCommand(double degrees) {
    return new InstantCommand(() -> setDegrees(degrees)).withName("Set Degrees");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty(
        "Test Enable", () -> m_testModeEnabled, (val) -> m_testModeEnabled = val);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    addChild("Controller", m_controller);
    addChild("Encoder", m_encoder);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
    builder.addDoubleProperty("Output", () -> m_sparkMax.getAppliedOutput(), null);
  }

  @Override
  public void setTrackingFunction(DoubleSupplier trackingSupplier) {
    m_supplier = trackingSupplier;
  }

  @Override
  public void enableTracking() {
    if (m_supplier == null) {
      Logger.tag("Turret").warn("Tracking enabled, but no tracking function is defined!");
    } else {
      Logger.tag("Turret").info("Tracking enabled");
    }
    m_trackingEnabled = true;
  }

  @Override
  public void disableTracking() {
    Logger.tag("Turret").info("Tracking disabled");
    m_trackingEnabled = false;
  }

  @Override
  public boolean isTrackingEnabled() {
    return m_trackingEnabled;
  }
}
