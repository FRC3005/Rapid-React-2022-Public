package lib;

import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import frc.lib.sim.LimelightSim;
import frc.lib.util.LinearInterpolatedTable2d;
import frc.lib.util.TestUtils;
import frc.lib.vendor.sensor.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import org.junit.*;

public class HoodTest {
  private static final double kEpsilon = 1E-9;

  private final Hood m_hood = new Hood();

  private final Limelight m_limelight =
      new Limelight(
          Constants.Vision.kCameraAngleDegrees,
          Constants.Vision.kCameraHeightMeters,
          Constants.Vision.kTargetHeightMeters);

  private final LimelightSim m_limelightSim = new LimelightSim();

  // spotless:off
  private final LinearInterpolatedTable2d kHoodLookup =
      new LinearInterpolatedTable2d()
      .withPair(2, 46)
      .withPair(4, 48)
      .withPair(6, 50);
  // spotless:on

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);

    m_limelightSim.reset();
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  void enableAndSetTest() {
    m_hood.set(10);
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertFalse(m_hood.isEnabled());
    assertEquals(0.0, m_hood.get(), kEpsilon);
    m_hood.enable();
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(10.0, m_hood.get(), kEpsilon);
    assertTrue(m_hood.isEnabled());
    m_hood.disable();
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(0.0, m_hood.get(), kEpsilon);
    assertFalse(m_hood.isEnabled());
  }

  void trackingFunction() {
    m_hood.enable();
    m_hood.set(10.0);
    m_hood.setTrackingFunction(() -> 20.0);
    m_hood.enableTracking();
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(20.0, m_hood.get(), kEpsilon);
    assertTrue(m_hood.isEnabled());
    m_hood.disableTracking();
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(10.0, m_hood.get(), kEpsilon);
    assertTrue(m_hood.isEnabled());
  }

  void limelightCommand() {
    m_hood.enable();
    m_hood.set(10.0);
    m_hood.setTrackingFunction(() -> kHoodLookup.get(m_limelight.getDistanceFromTargetMeters()));
    m_hood.enableTracking();
    m_limelightSim.setOffsetX(10.0);
    m_limelightSim.setOffsetY(10.0);
    m_limelightSim.setTargetArea(10.0);
    m_limelightSim.setValid(true);
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(46.35, m_hood.get(), 0.1);
    assertTrue(m_hood.isEnabled());
    m_hood.disableTracking();
    m_hood.set(10.0);
    TestUtils.subsystemTimeAdvance(1.0, m_hood);
    assertEquals(10.0, m_hood.get(), kEpsilon);
  }

  // Stack all tests into one @Test case due to duplicate initialization
  // of SparkMax objects which throws an error.
  @Test
  public void runTests() {
    enableAndSetTest();
    trackingFunction();
    limelightCommand();
  }
}
