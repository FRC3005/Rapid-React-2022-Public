package lib;

import static org.junit.Assert.*;

import org.junit.*;

public class TestTemplateTest {
  private static final double kEpsilon = 1E-9;

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void exampleTest() {
    int myTestValue = 4;

    double myDoubleValue = 17.17 / 3005.0;
    // Run all assertions even if they fail.
    assertTrue("This passes if true", true);
    assertEquals("This passes if these two are equal, expected goes 2nd", 4, myTestValue);
    assertEquals("Doubles don't have infinite precision", 0.00571381, myDoubleValue, kEpsilon);
  }
}
