// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.tinylog.Logger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final Elevator m_elevator = new Elevator(Constants.Elevator.kCanId);
  private final DoubleSolenoid m_spearSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          Constants.Spears.kFwdSolenoidChannel,
          Constants.Spears.kRevSolenoidChannel);

  public boolean m_testModeEnable = false;
  public boolean m_testSolenoidState = false;

  /** Creates a new Climber. */
  public Climber() {
    m_spearSolenoid.set(Value.kForward);
  }

  public void stop() {
    m_elevator.stop();
  }

  public void elevatorManual(double dutyCycle) {
    m_elevator.setOutput(dutyCycle);
  }

  public boolean isClimbing() {
    return m_elevator.getPosition() > (Constants.Elevator.kMidRungPosition / 3);
  }

  public void testModePeriodic() {
    if (m_testModeEnable) {
      m_elevator.testModePeriodic();
    }
  }

  public void resetSpears() {
    m_spearSolenoid.set(Value.kForward);
  }

  public Command prepareClimber() {
    return new FunctionalCommand(
      () -> {
        m_elevator.setPosition(Constants.Elevator.kMidRungPosition);
        // m_swords.setPosition(Constants.Sword.kMidRungPosition);
        m_spearSolenoid.set(Value.kReverse);
      },
      () -> {},
      (interrupted) -> {
        if (interrupted) {
          Logger.tag("Climber").warn("Interrupted climb deploy command");
        }
        m_elevator.stop();
      },
      () -> {
        return m_elevator.atSetpoint();
      },
      this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Test Mode Enable", () -> m_testModeEnable, (val) -> m_testModeEnable = val);
    builder.addBooleanProperty("Test Solenoid State (Disabled)", () -> m_testSolenoidState, (val) -> m_testSolenoidState = val);
  }
}
