// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.SetpointTracking;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import java.util.function.DoubleSupplier;
import org.tinylog.Logger;

public class Shooter extends SubsystemBase implements SetpointTracking {
  private final Flywheel m_flywheel = new Flywheel();
  private final Accelerator m_accelerator = new Accelerator();

  // Not owned by this subsystem
  private final Indexer m_indexer;

  private double m_shotRpm = Constants.Shooter.kDefaultShotRpm;
  private double m_manualShotRpm = m_shotRpm;
  private double m_idleRpm = Constants.Shooter.kIdleRpm;
  private DoubleSupplier m_rpmSupplier = null;
  private boolean m_trackingEnabled = false;
  private boolean m_isShooting = false;

  private final double kMinShotRpm = 2000.0;

  /** Creates a new Shooter. */
  public Shooter(Indexer indexer) {
    m_indexer = indexer;
    setDefaultCommand(idleCommand());
  }

  public void setShotRpm(double rpm) {
    // Check this condition to prevent requirement of hitting the
    // periodic() function before calling spin up or shoot commands
    if (!m_trackingEnabled) {
      m_shotRpm = rpm;
    }
    m_manualShotRpm = rpm;
  }

  public void setIdleRpm(double rpm) {
    m_idleRpm = rpm;
  }

  public void enableTracking() {
    if (m_rpmSupplier == null) {
      Logger.tag("Shooter").warn("Tracking enabled, but no tracking function is defined!");
    } else {
      Logger.tag("Shooter").info("Tracking enabled");
    }
    m_trackingEnabled = true;
  }

  public void disableTracking() {
    Logger.tag("Shooter").info("Tracking disabled");
    m_trackingEnabled = false;
    m_shotRpm = m_manualShotRpm;
  }

  public boolean isTrackingEnabled() {
    return m_trackingEnabled;
  }

  public boolean isShooting() {
    return m_isShooting;
  }

  public Command spinUpCommand() {
    return new RunCommand(
            () -> {
              m_flywheel.setRPM(m_shotRpm);
              m_accelerator.setRPM(5000);
            },
            m_flywheel,
            m_accelerator,
            this)
        .withName("Spin Up Command");
  }

  public Command shootCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(
                    () -> {
                      m_flywheel.setRPM(m_shotRpm);
                      m_accelerator.setRPM(5000);
                    },
                    m_flywheel,
                    m_accelerator,
                    this)
                .alongWith(m_indexer.adjustCommand()),
            new WaitUntilCommand(m_flywheel::atSpeed),
            new InstantCommand(
                () -> {
                  m_isShooting = true;
                }),
            new RunCommand(
                    () -> {
                      m_accelerator.enableFeeder();
                    },
                    m_flywheel,
                    m_accelerator,
                    this)
                .raceWith(m_indexer.shootCommand()))
        .withName("Shoot");
  }

  // TODO: Better commads... This is kind of a hack for auton
  public Command postShootCommand() {
    return new InstantCommand(
        () -> {
          m_isShooting = false;
          m_accelerator.disableFeeder();
        });
  }

  public Command idleCommand() {
    return new RunCommand(
            () -> {
              // Always run at shot RPM?
              m_flywheel.setRPM(m_shotRpm);
              m_accelerator.disableFeeder();
              m_accelerator.setRPM(m_idleRpm);
              m_isShooting = false;
            },
            m_flywheel,
            m_accelerator,
            this)
        .withName("Idle");
  }

  @Override
  public void periodic() {
    if (m_trackingEnabled && m_rpmSupplier != null) {
      m_shotRpm = m_rpmSupplier.getAsDouble();
    } else {
      m_shotRpm = m_manualShotRpm;
    }

    if (m_shotRpm < kMinShotRpm) {
      m_shotRpm = kMinShotRpm;
    }
  }

  public void testModePeriodic() {
    m_accelerator.testModePeriodic();
    m_flywheel.testModePeriodic();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Flywheel Setpoint", () -> m_shotRpm, null);
  }

  public Flywheel getFlywheel() {
    return m_flywheel;
  }

  public Accelerator getAccelerator() {
    return m_accelerator;
  }

  @Override
  public void setTrackingFunction(DoubleSupplier trackingSupplier) {
    m_rpmSupplier = trackingSupplier;
  }
}
