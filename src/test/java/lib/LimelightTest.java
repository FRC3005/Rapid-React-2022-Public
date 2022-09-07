package lib;

import static org.junit.Assert.*;

import frc.lib.sim.LimelightSim;
import frc.lib.vendor.sensor.Limelight;
import org.junit.*;

public class LimelightTest {
  private static final double kEpsilon = 1E-9;

  // Limelight mounted 20 degrees past normal, 1 meter off the ground, target of 5m off the ground
  private static final Limelight m_limelight = new Limelight(20, 1, 5);
  private static final LimelightSim m_sim = new LimelightSim();

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
    m_sim.reset();
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void headOnDistance() {
    m_sim.setOffsetX(0.0);
    m_sim.setOffsetY(20.0);
    m_sim.setValid(true);

    assertEquals(4.767, m_limelight.getDistanceFromTargetMeters(), 0.001);
    assertEquals(4.767, m_limelight.getTargetTranslation().getNorm(), 0.001);
  }

  @Test
  public void distanceFromAnAngle() {
    m_sim.setOffsetX(15.0);
    m_sim.setOffsetY(20.0);
    m_sim.setValid(true);

    assertEquals(4.767, m_limelight.getDistanceFromTargetMeters(), 0.001);
    assertNotEquals(
        m_limelight.getDistanceFromTargetMeters(),
        m_limelight.getTargetTranslation().getNorm(),
        0.001);

    assertTrue(
        "Translation corrects for image distortion and should be greater than the naive approach",
        m_limelight.getDistanceFromTargetMeters() < m_limelight.getTargetTranslation().getNorm());

    /**
     * Frames are tricky here. Camera X is left/right, and Y is up/down. For the transform, X is
     * forward and Y is left right, with left being positive.
     */
    assertTrue("Transform 'Y' should be negative", m_limelight.getTargetTranslation().getY() < 0.0);
  }
}
