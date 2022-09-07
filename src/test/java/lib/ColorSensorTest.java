package lib;

import static org.junit.Assert.*;

import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.lib.sim.ColorSensorV3Sim;
import frc.lib.util.ThreadUtils;
import frc.robot.subsystems.indexer.DualColorSensorThread;
import org.junit.*;

public class ColorSensorTest {
  private DualColorSensorThread m_sensor;
  private ColorSensorV3Sim m_sensorSim;

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
    m_sensor = new DualColorSensorThread(Port.kMXP);
    m_sensorSim = new ColorSensorV3Sim(Port.kMXP);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
    m_sensor.close();
  }

  @Test
  public void threadTest() {
    m_sensorSim.setColor(100, 200, 300, 400);
    m_sensorSim.setProximity(500);
    ThreadUtils.sleep(50);
    RawColor c = m_sensor.getRawColor(0);
    assertEquals(c.red, 100);
  }
}
