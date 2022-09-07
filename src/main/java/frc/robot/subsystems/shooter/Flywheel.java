// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

public class Flywheel extends SubsystemBase {
  private final FlywheelSim m_flywheelSim;
  private final SparkMax m_sparkMax;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private final Controller m_controller;
  private final SimpleMotorFeedforward m_feedforward;
  private final BangBangController m_bangbang = new BangBangController();
  private double m_setpoint = 0.0;
  private boolean m_testEnable = false;
  private final Encoder m_encoder;

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors +=
        SparkMaxUtils.check(
            sparkMax
                .getEncoder()
                .setVelocityConversionFactor(1 / Constants.Shooter.Flywheel.kGearRatio));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocity);
    return errors == 0;
  }

  /** Creates a new Shooter. */
  public Flywheel() {
    m_sparkMax =
        new SparkMax(Constants.Shooter.Flywheel.kprimarySparkCanId)
            .withInitializer(Flywheel::sparkMaxInitializer);
    m_controller = m_sparkMax.velocityController(Constants.Shooter.Flywheel.kPidGains, 0.0, 1.0);
    m_feedforward = Constants.Shooter.Flywheel.kFeedforward;
    m_encoder = m_sparkMax.builtinEncoder();
    m_sparkMaxSim = new CANSparkMaxSim1(m_sparkMax);
    m_flywheelSim =
        new FlywheelSim(
            Constants.Shooter.Flywheel.kSimMotor,
            Constants.Shooter.Flywheel.kGearRatio,
            Constants.Shooter.Flywheel.kInertia);
  }

  public void setRPM(double rpm) {
    m_setpoint = rpm;
  }

  public boolean atSpeed() {
    // Basic implementation, used to turn on feeder
    return m_encoder.getVelocity()
        >= ((1.0 - Constants.Shooter.Flywheel.kAllowedVelocityError) * m_setpoint);
  }

  @Override
  public void periodic() {
    if (m_setpoint > 0.0) {
      m_controller.setReference(
          m_setpoint, m_encoder.getVelocity(), m_feedforward.calculate(m_setpoint));
    } else {
      // Put the device into coast
      m_sparkMax.set(0.0);
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
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (m_testEnable) {
      if (m_setpoint == 0.0) {
        m_sparkMax.set(0.0);
      } else {
        m_controller.setReference(
            m_setpoint, m_encoder.getVelocity(), m_feedforward.calculate(m_setpoint));
      }
    } else {
      m_sparkMax.set(0.0);
    }
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

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("Spark Max Controller", m_controller);
    builder.addBooleanProperty("Test Enable", () -> m_testEnable, (val) -> m_testEnable = val);
    builder.addDoubleProperty(
        "Setpoint",
        () -> {
          return m_setpoint;
        },
        (val) -> m_setpoint = val);
    builder.addDoubleProperty("Velocity", () -> m_encoder.getVelocity(), null);
    builder.addDoubleProperty("Appiled Output", () -> m_sparkMax.getAppliedOutput(), null);
    builder.addBooleanProperty("At Speed?", () -> atSpeed(), null);
  }

  public double getRPM() {
    return m_encoder.getVelocity();
  }
}
