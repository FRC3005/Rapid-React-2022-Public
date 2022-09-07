// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Controller;
import frc.lib.electromechanical.Encoder;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;

public class Accelerator extends SubsystemBase {
  private final FlywheelSim m_flywheelSim;
  private final SparkMax m_sparkMax;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private final Controller m_controller;
  private final SimpleMotorFeedforward m_feedforward;
  private double m_setpoint = 0.0;
  private boolean m_feederEnabled = false;
  private boolean m_testEnable = false;
  private final Encoder m_encoder;

  private final SparkMax m_feederSparkMax;
  private final CANSparkMaxSim1 m_feederSparkMaxSim;
  private final FlywheelSim m_feederSim;
  private final double kFeederVoltage = Constants.Shooter.Accelerator.kFeederVoltage;
  private double m_testFeederVoltage = 0.0;

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1.0, 1.0));
    sparkMax.setInverted(Constants.Shooter.Accelerator.kIntert);
    sparkMax.setInverted(Constants.Shooter.Accelerator.kIntert);
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocity);
    return errors == 0;
  }

  /** Creates a new Shooter. */
  public Accelerator() {
    m_sparkMax =
        new SparkMax(Constants.Shooter.Accelerator.kprimarySparkCanId)
            .withInitializer(Accelerator::sparkMaxInitializer);
    m_controller = m_sparkMax.velocityController(Constants.Shooter.Accelerator.kPidGains, 0.0, 1.0);
    m_feedforward = Constants.Shooter.Accelerator.kFeedforward;
    m_encoder = m_sparkMax.builtinEncoder();
    m_sparkMaxSim = new CANSparkMaxSim1(m_sparkMax);
    m_flywheelSim =
        new FlywheelSim(
            Constants.Shooter.Accelerator.kSimMotor,
            Constants.Shooter.Accelerator.kGearRatio,
            Constants.Shooter.Accelerator.kInertia);

    // Feeder Init
    m_feederSparkMax =
        new SparkMax(Constants.Shooter.Accelerator.kFeederCanId)
            .withInitializer(
                (sparkMax, isInit) -> {
                  sparkMax.setInverted(Constants.Shooter.Accelerator.kFeederInvert);
                  sparkMax.setInverted(Constants.Shooter.Accelerator.kFeederInvert);
                  sparkMax.setIdleMode(IdleMode.kBrake);
                  SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
                  return true;
                });
    m_feederSparkMaxSim = new CANSparkMaxSim1(m_feederSparkMax);
    m_feederSim =
        new FlywheelSim(DCMotor.getNEO(1), 1.0, Constants.Shooter.Accelerator.kFeederInertia);
  }

  public void setRPM(double rpm) {
    m_setpoint = rpm;
  }

  public void enableFeeder() {
    m_feederEnabled = true;
  }

  public void disableFeeder() {
    m_feederEnabled = false;
  }

  @Override
  public void periodic() {
    if (m_setpoint == 0.0) {
      m_sparkMax.set(0.0);
    } else {
      m_controller.setReference(
          m_setpoint, m_encoder.getVelocity(), m_feedforward.calculate(m_setpoint));
    }

    if (m_feederEnabled) {
      m_feederSparkMax.setVoltage(kFeederVoltage);
    } else {
      // Put the device into coast
      m_feederSparkMax.setVoltage(-6.0);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_sparkMaxSim.enable();
    m_sparkMaxSim.iterate(
        m_flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
    m_flywheelSim.setInputVoltage(
        m_sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_flywheelSim.update(0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
    m_sparkMaxSim.setMotorCurrent(m_flywheelSim.getCurrentDrawAmps());

    // Feeder sim
    m_feederSparkMaxSim.enable();
    m_feederSparkMaxSim.iterate(
        m_feederSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
    m_feederSim.setInputVoltage(
        m_feederSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_feederSim.update(0.02);
    m_feederSparkMaxSim.setMotorCurrent(m_feederSim.getCurrentDrawAmps());
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (m_testEnable) {
      m_controller.setReference(
          m_setpoint, m_encoder.getVelocity(), m_feedforward.calculate(m_setpoint));
      m_feederSparkMax.setVoltage(m_testFeederVoltage);
    } else {
      m_sparkMax.set(0.0);
      m_feederSparkMax.set(0.0);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("Spark Max Controller", m_controller);
    builder.addBooleanProperty("enable", () -> m_testEnable, (val) -> m_testEnable = val);
    builder.addDoubleProperty("setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    builder.addDoubleProperty(
        "Feeder Test Voltage", () -> m_testFeederVoltage, (val) -> m_testFeederVoltage = val);
    builder.addDoubleProperty("Accelerator Velocity", () -> m_encoder.getVelocity(), null);
  }

  /**
   * Set the simulated velocity of the sytem. Useful to test recovery time after shooting a game
   * piece
   *
   * @param velocity shooter velocity in RPM
   */
  public void setSimVelocity(double velocity) {
    m_flywheelSim.setState(
        new MatBuilder<>(Nat.N1(), Nat.N1())
            .fill(Units.rotationsPerMinuteToRadiansPerSecond(velocity)));
  }

  public double getRPM() {
    return m_encoder.getVelocity();
  }
}
