// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private boolean m_intakeState = false;
  private boolean m_testModeEnable = false;
  private final CANSparkMaxSim1 m_sparkMaxSim;
  private final SparkMax m_sparkMax;

  private final DoubleSolenoid m_solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          Constants.Intake.kFwdSolenoidChannel,
          Constants.Intake.kRevSolenoidChannel);
  /** Creates a new Intake. */
  public Intake() {
    m_sparkMax =
        new SparkMax(Constants.Intake.kCanId)
            .withInitializer(
                (sparkmax, init) -> {
                  SparkMax.setFrameStrategy(sparkmax, FrameStrategy.kNoFeedback);
                  return true;
                });
    m_sparkMaxSim = new CANSparkMaxSim1(m_sparkMax);
    setDefaultCommand(new RunCommand(this::intakeIn, this));
  }

  public void intakeOut() {
    m_sparkMax.setVoltage(Constants.Intake.kIntakeVoltage);
    m_solenoid.set(Value.kForward);
    m_intakeState = true;
  }

  public void intakeOutRollersReversed() {
    m_sparkMax.setVoltage(-Constants.Intake.kIntakeVoltage);
    m_solenoid.set(Value.kForward);
    m_intakeState = true;
  }

  public void intakeIn() {
    m_sparkMax.set(0.0);
    m_solenoid.set(Value.kReverse);
    m_intakeState = false;
  }

  public Command intakeOutCommand() {
    return new RunCommand(this::intakeOut, this).withName("Intake Out");
  }

  public Command intakeInCommand() {
    return new InstantCommand(this::intakeIn, this).withName("Intake In");
  }

  public Command intakeOutReversedCommand() {
    return new RunCommand(this::intakeOutRollersReversed, this).withName("Intake Out Reverse");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    m_sparkMaxSim.iterate(RobotController.getBatteryVoltage(), 0.02);
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (m_testModeEnable && m_intakeState) {
      intakeOut();
    } else {
      intakeIn();
    }
  }

  public boolean intakeToggle() {
    if (m_intakeState) {
      intakeIn();
    } else {
      intakeOut();
    }

    return m_intakeState;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty(
        "Test Enable", () -> m_testModeEnable, (val) -> m_testModeEnable = val);
    builder.addBooleanProperty("Intake State", () -> m_intakeState, (val) -> m_intakeState = val);
    addChild("Solenoid", m_solenoid);
  }
}
