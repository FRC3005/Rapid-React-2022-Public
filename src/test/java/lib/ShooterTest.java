package lib;

import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import frc.lib.sim.LimelightSim;
import frc.lib.util.LinearInterpolatedTable2d;
import frc.lib.util.TestUtils;
import frc.lib.vendor.sensor.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Accelerator;
import frc.robot.subsystems.shooter.Shooter;
import org.junit.*;

public class ShooterTest {
  private static final double kEpsilon = 1E-9;
  private Indexer m_indexer = new Indexer();
  private Shooter m_shooter = new Shooter(m_indexer);

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  private void acceleratorSetRpmTest() {
    Accelerator accelerator = m_shooter.getAccelerator();
    accelerator.setRPM(4000);
    accelerator.enableFeeder();
    TestUtils.subsystemTimeAdvance(0.5, accelerator);
    assertEquals("Shooter gets up to speed fast enough", 4000, accelerator.getRPM(), 50);
    accelerator.disableFeeder();
    accelerator.setRPM(0.0);
    TestUtils.subsystemTimeAdvance(10.0, accelerator);
    assertEquals("Shooter slows down after disable", 0.0, accelerator.getRPM(), 0.1);
  }

  private void acceleratorFeedTest() {
    Accelerator accelerator = m_shooter.getAccelerator();
    accelerator.setRPM(4000);
    accelerator.enableFeeder();
    TestUtils.subsystemTimeAdvance(2.0, accelerator);
    assertEquals("Shooter gets up to speed fast enough", 4000, accelerator.getRPM(), 50);
    accelerator.setSimVelocity(3500);
    TestUtils.subsystemTimeAdvance(0.4, accelerator);
    assertEquals("Shooter recovers fast enough", 4000, accelerator.getRPM(), 50);
    accelerator.disableFeeder();
    accelerator.setRPM(0.0);
    TestUtils.subsystemTimeAdvance(10.0, accelerator);
    assertEquals("Shooter slows down after disable", 0.0, accelerator.getRPM(), 0.1);
  }

  private void FlywheelSetRpmTest() {
    TestUtils.subsystemTimeAdvance(1.0, m_shooter.getFlywheel());
    assertEquals("Disabled on start", 0.0, m_shooter.getFlywheel().getRPM(), 0.1);
    m_shooter.getFlywheel().setRPM(4000);
    TestUtils.subsystemTimeAdvance(2.0, m_shooter.getFlywheel());
    assertEquals(
        "Shooter gets up to speed fast enough", 4000, m_shooter.getFlywheel().getRPM(), 50);
    m_shooter.getFlywheel().setRPM(0);
    TestUtils.subsystemTimeAdvance(10.0, m_shooter.getFlywheel());
    assertEquals("Shooter slows down after disable", 0.0, m_shooter.getFlywheel().getRPM(), 0.1);
  }

  private void FlywheelFeedTest() {
    m_shooter.getFlywheel().setRPM(4000);
    TestUtils.subsystemTimeAdvance(2.0, m_shooter.getFlywheel());
    assertEquals(
        "Shooter gets up to speed fast enough", 4000, m_shooter.getFlywheel().getRPM(), 50);
    m_shooter.getFlywheel().setSimVelocity(3500);
    TestUtils.subsystemTimeAdvance(1.8, m_shooter.getFlywheel());
    assertEquals("Shooter recovers fast enough", 4000, m_shooter.getFlywheel().getRPM(), 50);
    m_shooter.getFlywheel().setRPM(0);
    TestUtils.subsystemTimeAdvance(10.0, m_shooter.getFlywheel());
    assertEquals("Shooter slows down after disable", 0.0, m_shooter.getFlywheel().getRPM(), 0.1);
  }

  private void trackingTest() {
    Limelight limelight =
        new Limelight(
            Constants.Vision.kCameraAngleDegrees,
            Constants.Vision.kCameraHeightMeters,
            Constants.Vision.kTargetHeightMeters);
    // spotless:off
    LinearInterpolatedTable2d lookup =
        new LinearInterpolatedTable2d()
        .withPair(2, 2500)
        .withPair(4, 2700)
        .withPair(6, 2800);
    // spotless:on
    LimelightSim limelightSim = new LimelightSim();

    m_shooter.setTrackingFunction(() -> lookup.get(limelight.getDistanceFromTargetMeters()));
    m_shooter.enableTracking();
    limelightSim.setOffsetX(10.0);
    limelightSim.setOffsetY(10.0);
    limelightSim.setTargetArea(10.0);
    limelightSim.setValid(true);
    m_shooter.spinUpCommand().execute();
    TestUtils.subsystemTimeAdvance(
        1.0, m_shooter, m_shooter.getFlywheel(), m_shooter.getAccelerator());
    assertEquals(2780, m_shooter.getFlywheel().getRPM(), 50);
    assertTrue(m_shooter.isTrackingEnabled());
    m_shooter.disableTracking();
    TestUtils.subsystemTimeAdvance(
        2.0, m_shooter, m_shooter.getFlywheel(), m_shooter.getAccelerator());
    m_shooter.setShotRpm(4000);
    m_shooter.spinUpCommand().execute();
    TestUtils.subsystemTimeAdvance(
        2.0, m_shooter, m_shooter.getFlywheel(), m_shooter.getAccelerator());
    assertEquals(4000, m_shooter.getFlywheel().getRPM(), 50);
    assertFalse(m_shooter.isTrackingEnabled());
  }

  @Test
  public void runTest() {
    // Spark Max doesn't seem to close between tests, so their resources
    // aren't cleaned up, and subsuquent allocations flag an error. Instead
    // run everything as one test
    FlywheelFeedTest();
    FlywheelSetRpmTest();
    acceleratorFeedTest();
    acceleratorSetRpmTest();
    trackingTest();
  }
}
