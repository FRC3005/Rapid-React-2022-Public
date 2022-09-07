// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.controller.Controller;
import frc.lib.electromechanical.Encoder;
import frc.lib.util.SendableHelper;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;

public class IndexerRoller implements Sendable {
  private final SparkMax m_motor;
  private final Controller m_thetaController;
  private final Encoder m_encoder;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private final LinearFilter m_filter = LinearFilter.singlePoleIIR(0.75, 0.02);
  private final FlywheelSim m_sim = new FlywheelSim(DCMotor.getNeo550(1), 1.0, 0.00003973);
  private double m_filteredCurrent = 0.0;
  private boolean m_stalled = false;

  /** Creates a new Indexer Roller */
  public IndexerRoller() {
    m_motor =
        new SparkMax(Constants.Indexer.kIndexerCanId)
            .withInitializer(
                (sparkMax, isInit) -> {
                  int errors = 0;
                  errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
                  errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
                  errors +=
                      SparkMaxUtils.check(
                          SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
                              sparkMax.getEncoder(), Constants.Indexer.kIndexerGearRatio));
                  SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kPosition);
                  return errors == 0;
                });
    m_sparkMaxSim = new CANSparkMaxSim1(m_motor, m_sim);
    m_thetaController = m_motor.positionController(Constants.Indexer.kIndexerPositionGains);
    m_encoder = m_motor.builtinEncoder();
  }

  public void set(double voltage) {
    m_motor.setVoltage(voltage);
  }

  /** Stop running the vbelt. */
  public void stop() {
    m_motor.set(0.0);
  }

  public void rotateBy(double radiansToRotate) {
    m_thetaController.setReference(
        radiansToRotate + m_encoder.getPosition(), m_encoder.getPosition());
  }

  /**
   * Check whether the vbelt is running
   *
   * @return true if the vbelt is running
   */
  public boolean running() {
    return Math.abs(m_motor.getAppliedOutput()) > 0.001;
  }

  protected void periodic() {

    m_filteredCurrent = m_filter.calculate(m_motor.getOutputCurrent());
    if (m_filteredCurrent > Constants.Indexer.kCurrentThreshold) {
      m_stalled = true;
    }

    if (m_filteredCurrent
        < (Constants.Indexer.kCurrentThreshold - Constants.Indexer.kCurrentHysterisis)) {
      m_stalled = false;
    }
  }

  public boolean stalled() {
    return m_stalled;
  }

  protected void simulationPeriodic(double dt) {
    m_sparkMaxSim.iterate(RobotController.getBatteryVoltage(), dt);

    if (m_simStall) {
      setSimVelocity(0.0);
    }
  }

  public void setSimVelocity(double velocity) {
    m_sim.setState(
        new MatBuilder<>(Nat.N1(), Nat.N1())
            .fill(Units.rotationsPerMinuteToRadiansPerSecond(velocity)));
  }

  private boolean m_simStall = false;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Running", () -> running(), null);
    builder.addDoubleProperty("Indexer Current", () -> m_motor.getOutputCurrent(), null);
    builder.addDoubleProperty("Filtered Current", () -> m_filteredCurrent, null);
    builder.addBooleanProperty("Indexer Stalled", this::stalled, null);

    if (RobotBase.isSimulation()) {
      builder.addBooleanProperty("Sim Stall", () -> m_simStall, (val) -> m_simStall = val);
    }
    SendableHelper.addChild(builder, this, m_thetaController, "Theta Controller");
    builder.addDoubleProperty(
        "Roller Theta Degrees", () -> Units.radiansToDegrees(m_encoder.getPosition()), null);
  }
}
