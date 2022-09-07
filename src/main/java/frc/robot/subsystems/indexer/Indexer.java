// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.vendor.sensor.GY53;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.tinylog.Logger;

public class Indexer extends SubsystemBase {
  private static final double kFirstSensorHasBallDistance = 120;//100 was used at states
  private static final double kSensorDistanceTarget = 120;
  private static final double kSecondSensorHasBallDistance = -1000;
  private static final double kFirstSensorNoBallThreshold = 250;
  private static final double kSecondSensorNoBallThreshold = 250;
  private final FlappyRoller m_flappy = new FlappyRoller();
  private final IndexerRoller m_roller = new IndexerRoller();
  private final GY53 m_firstSensor = new GY53(Constants.Indexer.kFirstSensorDIO);
  private final GY53 m_secondSensor = new GY53(Constants.Indexer.kSecondSensorDIO);
  private boolean m_sensorOverride = false;
  // private final PicoColorSensor m_picoColorSensor = new PicoColorSensor();
  private Pair<ColorDistance, ColorDistance> m_currentBallState;
  // TODO: We may use a pi pico based solution for below
  // private final ColorDistanceSensor m_firstSensor = new FakeColorDistance(10.0, 0);
  // private final ColorDistanceSensor m_secondSensor = new FakeColorDistance(10.0, 1);
  private final double kWaitAfterIntakeSeconds = 3.0;
  private final double kFirstShotTimeoutSeconds = 0.0;

  // Test Mode Variable
  private boolean m_testModeEnable = false;
  private boolean m_vbeltLeftTest = false;
  private boolean m_vbeltRightTest = false;
  private double m_rollerTestVoltage = 0.0;

  // Track state for telemetry
  private String m_state = "Code Startup";

  /** Creates a new Indexer. */
  public Indexer() {
    m_currentBallState = Pair.of(new ColorDistance(), new ColorDistance());
    this.setDefaultCommand(IntakeIdle());
    // m_picoColorSensor.setDebugPrints(true);
  }

  /**
   * Internal commands are states in a state machine, scheduled based on public commands. Subsystem
   * periodic is called before commands, which allows sampling the color sensors first.
   */
  private class IntakeState extends CommandBase {
    private double m_startTime = 0.0;
    private final Runnable m_execute;
    private final BooleanSupplier m_isFinished;
    private final Consumer<Boolean> m_onEnd;

    public IntakeState(Runnable execute, BooleanSupplier isFinished, Subsystem s) {
      this(execute, isFinished, null, s);
    }

    public IntakeState(
        Runnable execute, BooleanSupplier isFinished, Consumer<Boolean> onEnd, Subsystem s) {
      m_onEnd = onEnd;
      m_execute = execute;
      m_isFinished = isFinished;
      addRequirements(s);
    }

    @Override
    public void initialize() {
      Logger.tag("IndexerState").info("Starting state {}", this.getName());
      m_startTime = Timer.getFPGATimestamp();
      m_state = this.getName();
    }

    @Override
    public void execute() {
      m_execute.run();
    }

    @Override
    public boolean isFinished() {
      return m_isFinished.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
      if (m_onEnd != null) {
        m_onEnd.accept(interrupted);
      }
      m_state = "";
      Logger.tag("IndexerState")
          .info(
              "Ending state {} after {} seconds{}",
              this.getName(),
              Timer.getFPGATimestamp() - m_startTime,
              interrupted ? " interrupted" : "");
    }
  }

  /**
   * Idle state is the state during normal operation. That is, the robot is just drive around with
   * or without game pieces. Indexer is off.
   */
  private final Command IntakeIdle() {
    return new IntakeState(
            () -> {
              m_roller.stop();
              m_flappy.stop();
            },
            () -> false,
            this)
        .withName("Intake Idle");
  }

  /** Intake until the first game peice is detected in place near the shooter. */
  private final Command IntakeFirst() {
    return new IntakeState(
            () -> {
              m_roller.set(Constants.Indexer.kIndexerForwardVoltage);
              m_flappy.run();
            },
            () -> {
              if (m_roller.stalled()) {
                Logger.tag("Indexer")
                    .debug(
                        "Intake First due to Stall, Ball distance {}",
                        m_firstSensor.getDistanceMillimeters());
              }
              return m_roller.stalled()
                  || m_firstSensor.getDistanceMillimeters() < kFirstSensorHasBallDistance;
            },
            this)
        .withName("Intake First");
  }

  /** Intake after a gamepeice is in the shooter until a second game peice is detected. */
  private final Command IntakeSecond() {
    return new IntakeState(
            () -> {
              m_roller.stop(); // set(Constants.Indexer.kIndexerForwardVoltage);
              m_flappy.run();
            },
            () -> {
              return false;
            },
            this)
        .withName("Intake Second");
  }

  /** The game peice should not be too close to the feeder rollers. Move it back slightly. */
  private final Command Adjust() {
    return new IntakeState(
            () -> {
              m_roller.stop();
              m_roller.rotateBy(Constants.Indexer.kIndexerAdjustRadians);
            },
            () -> {
              return true;
            },
            (interrupted) -> {},
            this)
        .withName("Adjust");
  }

  private final Command Unadjust() {
    return new IntakeState(
            () -> {
              m_roller.stop();
              m_roller.rotateBy(-1.5 * Constants.Indexer.kIndexerAdjustRadians);
            },
            () -> {
              return true;
            },
            (interrupted) -> {},
            this)
        .withName("Unadjust");
  }

  /**
   * Shoot the first game peice. This invloves only the indexer roller, so that an arbitrary amount
   * of time can be inserted between the two game peices. This may not be needed depending on the
   * characteristics of the shooter.
   */
  private final Command ShootFirst() {
    return new IntakeState(
            () -> {
              m_roller.set(Constants.Indexer.kIndexerShootVoltage);
              m_flappy.stop();
            },
            () -> {
              // return m_firstSensor.getDistanceMillimeters() > kFirstSensorNoBallThreshold;
              // Could use above, for now manually time it. Ideal case would be to check the
              // shooter RPM.
              return false;
            },
            this)
        .withName("Shoot First")
        .raceWith(new WaitCommand(kFirstShotTimeoutSeconds).withName("First Shot Timeout"));
  }

  /** Shoot the second game peice. This will turn on all motors to push the second ball through. */
  private final Command ShootSecond() {
    return new IntakeState(
            () -> {
              m_roller.set(Constants.Indexer.kIndexerShootVoltage);
              m_flappy.run();
            },
            () -> {
              // Could use sensors, but I think time is fine here, since the driver can tell if the
              // ball is shot, and its okay for this to keep running after shooting for a few
              // seconds.
              return false;
            },
            this)
        .withName("Shoot Second")
        .andThen(new WaitCommand(3.0).withName("Wait after Shoot"))
        .raceWith(new WaitCommand(5.0).withName("Shoot Second Timeout"));
  }

  /**
   * Reverse the indexer, both the feeder roller and the brush bar.
   *
   * @return command to be scheduled
   */
  private final Command ReverseIndexer() {
    return new IntakeState(
            () -> {
              m_roller.set(Constants.Indexer.kIndexerForwardVoltage * -1);
              m_flappy.reverse();
            },
            () -> false,
            this)
        .withName("Reverse Indexer");
  }

  /**
   * Create an intake command. This will start the intake process, which will run until two game
   * peices are in the robot, or cancelled.
   *
   * @return an intake command to be scheduled
   */
  public Command intakeCommand() {
    return new SequentialCommandGroup(
            IntakeFirst(), Adjust(), IntakeSecond(), new WaitCommand(kWaitAfterIntakeSeconds))
        .withName("Intake");
  }

  /**
   * Intake without waiting leaving on rollers for extra time
   *
   * @return an intake command to be scheduled.
   */
  public Command fastIntakeCommand() {
    return new SequentialCommandGroup(IntakeFirst(), IntakeSecond()).withName("Fast Intake");
  }

  public Command reverseIndexerCommand() {
    return ReverseIndexer();
  }

  /**
   * Create a shoot command. This will start the shooting proccess for the intake, which will push
   * the first gamepiece until it is out of the first position, then turn on all rollers until all
   * game peices are out of the robot.
   *
   * @return a shoot command to be scheduled
   */
  public Command shootCommand() {
    return new SequentialCommandGroup(ShootFirst(), ShootSecond()).withName("Shoot");
  }

  public Command intakeCancelCommand() {
    return new SequentialCommandGroup(new WaitCommand(kWaitAfterIntakeSeconds), IntakeIdle())
        .withName("Intake Cancel");
  }

  /**
   * Create a command to adjust the game peice before shooting.
   *
   * @return adjust command to be scheduled
   */
  public Command adjustCommand() {
    return Adjust();
  }

  public Command unadjustCommand() {
    return Unadjust();
  }

  /**
   * Create a stop command. This will stop all motors.
   *
   * @return a stop command to be scheduled
   */
  public Command stopCommand() {
    return IntakeIdle();
  }

  /**
   * Turn on all rollers until interrupted. No sensor feedback is used.
   *
   * @return a manual intake command
   */
  public Command intakeManualCommand() {
    return new RunCommand(
            () -> {
              m_roller.set(Constants.Indexer.kIndexerForwardVoltage);
              m_flappy.run();
            },
            this)
        .withName("Intake Manual");
  }

  public void intake() {
    CommandScheduler.getInstance().schedule(intakeCommand());
  }

  public void shoot() {
    CommandScheduler.getInstance().schedule(shootCommand());
  }

  public void stop() {
    CommandScheduler.getInstance().schedule(stopCommand());
  }

  public void intakeManual() {
    CommandScheduler.getInstance().schedule(intakeManualCommand());
  }

  @Override
  public void periodic() {
    /** Collect current state of sensors */
    m_currentBallState.getFirst().distance = m_firstSensor.getDistanceMillimeters();
    m_currentBallState.getSecond().distance = m_secondSensor.getDistanceMillimeters();

    m_roller.periodic();
  }

  @Override
  public void simulationPeriodic() {
    m_flappy.simulationPeriodic(0.02);
    m_roller.simulationPeriodic(0.02);
  }

  public void testModePeriodic() {
    if (RobotBase.isSimulation()) {
      periodic();
      simulationPeriodic();
    }

    if (m_testModeEnable) {
      if (m_vbeltLeftTest) {
        m_flappy.run();
      } else {
        m_flappy.stop();
      }

      m_roller.set(m_rollerTestVoltage);
    }
  }

  /**
   * Return the amount of cargo in the bot
   *
   * @return 0, 1, or 2 cargo
   */
  public int cargoCount() {
    int result = 0;
    if (m_currentBallState.getFirst().distance < kFirstSensorHasBallDistance) {
      result++;
    }

    if (m_currentBallState.getSecond().distance < kSecondSensorHasBallDistance) {
      result++;
    }

    return result;
  }

  /**
   * Return the color of the next game piece in the bot (the one closest to the shooter) if one is
   * in that position.
   *
   * @return Color.kRed or Color.kBlue
   */
  public Optional<Color> nextColor() {
    if (m_currentBallState.getFirst().color == Color.kBlack) {
      return Optional.empty();
    } else {
      return Optional.of(m_currentBallState.getFirst().color);
    }
  }

  /**
   * Return a pair representing the game piece colors in the robot. The first value is the ball
   * closest to the shooter.
   *
   * @return Pair where first is the ball closest to the shooter and second is the ball closest to
   *     the intake. Colors are kBlue, kRed, or kBlack (none)
   */
  public Pair<ColorDistance, ColorDistance> getGamePieceColors() {
    return m_currentBallState;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setActuator(false);
    addChild("Flappy Roller", m_flappy);
    addChild("Indexer Roller", m_roller);
    builder.addStringProperty("State", () -> m_state.toString(), null);
    builder.addBooleanProperty(
        "Sensor Override", () -> m_sensorOverride, (val) -> m_sensorOverride = val);
    builder.addBooleanProperty(
        "Test Enable", () -> m_testModeEnable, (val) -> m_testModeEnable = val);
    builder.addBooleanProperty(
        "Test Vbelt Left", () -> m_vbeltLeftTest, (val) -> m_vbeltLeftTest = val);
    builder.addBooleanProperty(
        "Test Vbelt Right", () -> m_vbeltRightTest, (val) -> m_vbeltRightTest = val);
    builder.addDoubleProperty(
        "Test Roller Voltage", () -> m_rollerTestVoltage, (val) -> m_rollerTestVoltage = val);
    builder.addDoubleProperty("Cargo Count", this::cargoCount, null);
    addChild("First Sensor (Near Shooter)", m_firstSensor);
    addChild("Second Sensor (Near Intake)", m_secondSensor);
    builder.addDoubleProperty(
        "Index Roller Rotate Degrees",
        () -> 0.0,
        (val) -> m_roller.rotateBy(Units.degreesToRadians(val)));
  }
}
