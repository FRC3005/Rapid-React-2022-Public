package lib;

import static org.junit.Assert.*;

import org.junit.*;

public class GearboxTest {
  @Before // this method will run before each test
  public void setup() {
    // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  /*
  in -> a0
  a0 -> a1
  a1 -> a2
  a2 -> a3
  a3 -> out

  in [type=input]
  a0 [teeth=32]
  a1 [teeth=23]
  a2 [teeth=10]
  a3 [diameter=2]
  out [type=output]
  */
  @Test
  public void gearboxTest() {
    return;
    // GearboxBuilder g = new GearboxBuilder();
    /*
    g.addGear("a0", 32).isInput();
    g.addGear("a1", 23).coupledTo("a0");
    g.addGear("a2", 10).coupledTo("a1");
    g.addWheel("a3", 2).coupledTo("a2").isOutput();
    g.addGear("a4", 32).axleMounedTo("a1");
    */
    // g.addGear("a0", 32).isInput();
    // g.addGear("a1", 10).coupledTo("a0").isOutput();

    // Gearbox gb = g.build();
    // assertEquals(32.0 / 10.0, gb.outputRatio(), 0.001);
  }
}
