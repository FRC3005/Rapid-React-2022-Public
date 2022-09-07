// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;

public class FlappyRoller implements Sendable {
  private final SparkMax m_motor;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private double m_voltage = Constants.Indexer.kFlappyRollerVoltage;

  /**
   * Creates a new VBetls
   *
   * @param vbeltSide either Side.kLeft or Side.kRight
   */
  protected FlappyRoller() {
    m_motor =
        new SparkMax(Constants.Indexer.kFlappyRollerCanId)
            .withInitializer(
                (sparkMax, isInit) -> {
                  int errors = 0;
                  errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
                  sparkMax.setInverted(Constants.Indexer.kFlappyRollerInvert);
                  sparkMax.setInverted(Constants.Indexer.kFlappyRollerInvert);
                  errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
                  return errors == 0;
                });
    m_sparkMaxSim = new CANSparkMaxSim1(m_motor);
  }

  public void run() {
    m_motor.setVoltage(m_voltage);
  }

  public void runSlow() {
    m_motor.setVoltage(Constants.Indexer.kFlappyRollerSlowVoltage);
  }

  public void reverse() {
    m_motor.setVoltage(Constants.Indexer.kFlappyRollerVoltage * -1);
  }

  /** Stop running the vbelt. */
  public void stop() {
    m_motor.set(0.0);
  }

  /**
   * Check whether the vbelt is running
   *
   * @return true if the vbelt is running
   */
  public boolean running() {
    return m_motor.getAppliedOutput() > 0.0;
  }

  protected void simulationPeriodic(double dt) {
    m_sparkMaxSim.iterate(RobotController.getBatteryVoltage(), dt);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Running", () -> running(), null);
    builder.addDoubleProperty("Flappy Current", () -> m_motor.getOutputCurrent(), null);
    builder.addDoubleProperty("Flappy Voltage", () -> m_voltage, (val) -> m_voltage = val);
  }
}
