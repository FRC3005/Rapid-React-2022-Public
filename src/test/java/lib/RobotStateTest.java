package lib;

import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.sim.ADIS16470Sim;
import frc.lib.sim.LimelightSim;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;
import org.junit.*;

public class RobotStateTest {
  private static final double kEpsilon = 1E-9;
  private final ADIS16470 m_gyro = new ADIS16470();
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);
  private final ADIS16470Sim m_simGyro = new ADIS16470Sim(m_gyro);
  private final Limelight m_limelight = new Limelight(20, 1, 5);
  private final LimelightSim m_simLimelight = new LimelightSim();

  private final RobotState m_state = new RobotState(m_drive, m_limelight, m_gyro, () -> 0.0);

  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {}

  @Test
  public void goalFromVisionTarget() {
    m_simLimelight.setOffsetX(15.0);
    m_simLimelight.setOffsetY(20.0);
    m_simLimelight.setValid(true);

    Translation2d target = m_limelight.getTargetTranslation();

    assertEquals(4.767, m_limelight.getDistanceFromTargetMeters(), 0.001);
    assertEquals(5.018, target.getNorm(), 0.001);
    Translation2d updated = RobotState.centerOfGoalFromVisionTranslation(target);
    double targetAngle = Math.atan2(target.getY(), target.getX());
    double updatedAngle = Math.atan2(updated.getY(), updated.getX());
    assertEquals(
        "Angle should stay the same",
        Units.radiansToDegrees(targetAngle),
        Units.radiansToDegrees(updatedAngle),
        kEpsilon);
    assertEquals(5.018 + Constants.Field.kGoalRadius, updated.getNorm(), 0.001);
  }
}
