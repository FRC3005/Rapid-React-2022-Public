package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Controller;
import frc.lib.electromechanical.Encoder;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants;

import org.tinylog.Logger;

public class Elevator extends SubsystemBase {
  private final SparkMax m_motor;
  private final SparkMax m_follower;

  private final Controller m_controller;

  private final Encoder m_encoder;

  private final CANSparkMaxSim1 m_motorSim;

  private final ElevatorFeedforward m_feedforward;
  private double m_setpoint;
  private boolean m_closedLoop = false;

  private static Boolean leaderSparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) Constants.Elevator.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) Constants.Elevator.kReverseSoftLimit));
    // errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(sparkMax.setSmartCurrentLimit(80));
    errors += SparkMaxUtils.check(sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, false));
    errors += SparkMaxUtils.check(sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true));
    sparkMax.setInverted(true);
    sparkMax.setInverted(true);
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocityAndPosition, true);
    return errors == 0;
  }

  private static final Boolean followerSparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
    return errors == 0;
  }

  public Elevator(int canId) {
    m_motor = new SparkMax(canId).withInitializer(Elevator::leaderSparkMaxInitializer);
    m_controller = m_motor.positionController(Constants.Elevator.kPositionPID);
    
    m_follower = new SparkMax(Constants.Elevator.kFollowerCanId).withInitializer(Elevator::followerSparkMaxInitializer);
    m_follower.follow(m_motor, true);

    m_encoder = m_motor.builtinEncoder();
    m_encoder.setPosition(Constants.Elevator.kStartingPosition);
    m_setpoint = Constants.Elevator.kStartingPosition;
    m_feedforward = Constants.Elevator.kElevatorFeedForward;
    m_motorSim = new CANSparkMaxSim1(m_motor);
  }

  public void stop() {
    setOutput(0.0);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public boolean setPosition(double position) {
    if (position <= Constants.Elevator.kForwardSoftLimit
        && position >= Constants.Elevator.kReverseSoftLimit) {
      m_closedLoop = true;
      m_setpoint = position;
      return true;
    } else {
      Logger.tag("Elevator").warn("Attempting to set elevator to a bad position {}", position);
      return false;
    }
  }

  public boolean atSetpoint() {
    return Math.abs(m_encoder.getPosition() - m_setpoint) <= Constants.Elevator.kPositionTolerance;
  }

  public double getOutput() {
    return m_motor.getAppliedOutput();
  }

  public void setOutput(double dutyCycle) {
    m_closedLoop = false;
    m_motor.set(dutyCycle);
  }

  public void elevatorRunToStart() {
    setPosition(Constants.Elevator.kStartingPosition);
  }

  public void elevatorRunToMidRung() {
    setPosition(Constants.Elevator.kMidRungPosition);
  }

  @Override
  public void periodic() {
    if (m_closedLoop) {
      m_controller.setReference(
          m_setpoint,
          m_encoder.getPosition(),
          0.0); // m_feedforward.calculate(m_encoder.getVelocity())
    }
  }

  @Override
  public void simulationPeriodic() {
    m_motorSim.enable();
    m_motorSim.iterate(RobotController.getBatteryVoltage(), 0.02);
  }

  public void testModePeriodic() {
    periodic();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    addChild("Controller", m_controller);
    addChild("Encoder", m_encoder);
    builder.addDoubleProperty("Position", () -> getPosition(), null);
    builder.addDoubleProperty("Output", () -> getOutput(), null);
    builder.addBooleanProperty("Closed Loop", () -> m_closedLoop, (val) -> m_closedLoop = val);
  }
}
