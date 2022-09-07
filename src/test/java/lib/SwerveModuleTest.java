package lib;

import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.sim.ThroughBoreEncoderSim;
import frc.lib.vendor.sensor.ThroughBoreEncoder;
import frc.robot.subsystems.drive.NickSwerveModule;
import org.junit.*;

public class SwerveModuleTest {
  private static final double kEpsilon = 1E-9;
  private final ThroughBoreEncoder m_absEncoder = new ThroughBoreEncoder(0, 0, 2 * Math.PI, false);
  private final ThroughBoreEncoderSim m_encoderSim = new ThroughBoreEncoderSim(m_absEncoder);
  private final NickSwerveModule m_swerveModule = new NickSwerveModule(45, 46, m_absEncoder);

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  private void swerveWrap() {
    m_encoderSim.setAbsolutePosition(2.0);
    assertEquals(2.0, m_swerveModule.getState().angle.getRadians(), kEpsilon);
    m_swerveModule.setDesiredState(new SwerveModuleState(-1.0, new Rotation2d(2 * Math.PI + 2.0)));
    assertEquals(2.0, m_swerveModule.getDesiredState().angle.getRadians(), kEpsilon);

    m_encoderSim.setAbsolutePosition(0.0);
    m_swerveModule.setDesiredState(new SwerveModuleState(-1.0, new Rotation2d(2 * Math.PI)));
    assertEquals(0.0, m_swerveModule.getDesiredState().angle.getRadians(), kEpsilon);

    m_encoderSim.setAbsolutePosition(3.9);
    assertEquals(3.9, m_swerveModule.getState().angle.getRadians(), kEpsilon);
    m_swerveModule.setDesiredState(new SwerveModuleState(-1.0, new Rotation2d(4.0)));
    assertEquals(4.0, m_swerveModule.getDesiredState().angle.getRadians(), kEpsilon);
  }

  @Test
  public void swerveModuleTests() {
    swerveWrap();
  }
}
