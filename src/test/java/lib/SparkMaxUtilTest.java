package lib;

import static org.junit.Assert.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim1;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMaxMonitor;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import org.junit.*;

public class SparkMaxUtilTest {
  private static final double kEpsilon = 1E-9;

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void faultToStringTest() {
    short faults =
        (short)
            ((1 << CANSparkMax.FaultID.kBrownout.value)
                | (1 << CANSparkMax.FaultID.kDRVFault.value)
                | (1 << CANSparkMax.FaultID.kOtherFault.value));

    String result = SparkMaxUtils.faultWordToString(faults).strip();
    assertEquals("kBrownout kDRVFault kOtherFault", result);
  }

  @Test
  public void monitorTest() {
    SparkMax sparkMax = new SparkMax(59);
    CANSparkMaxSim1 sparkMaxSim = new CANSparkMaxSim1(sparkMax);
    SparkMaxMonitor monitor = new SparkMaxMonitor();
    monitor.add(sparkMax);
    sparkMaxSim.setFaults(CANSparkMax.FaultID.kIWDTReset, CANSparkMax.FaultID.kCANRX);
    monitor.periodic();
  }
}
