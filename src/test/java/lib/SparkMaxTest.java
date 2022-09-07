package lib;

import static org.junit.Assert.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.SparkMax;
import java.util.function.BiFunction;
import org.junit.*;

public class SparkMaxTest {

  @Before
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @After
  public void shutdown() throws Exception {}

  private static final BiFunction<CANSparkMax, Boolean, Boolean> initFunction =
      (CANSparkMax sparkMax, Boolean initializer) -> {
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(20);

        sparkMax.enableVoltageCompensation(12.12);
        return true;
      };

  @Test
  public void basicDriverTest() {
    SparkMax sparkMax = new SparkMax(4, MotorType.kBrushless).withInitializer(initFunction);

    int devId = sparkMax.getDeviceId();
    var tmp = sparkMax.getIdleMode();

    assertEquals(4, devId);

    assertEquals(IdleMode.kBrake, tmp);

    sparkMax.mutate(
        (spark, init) -> {
          spark.setIdleMode(IdleMode.kCoast);
          return true;
        });

    tmp = sparkMax.getIdleMode();
    assertEquals(IdleMode.kCoast, tmp);
  }
}
