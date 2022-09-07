package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.vendor.sensor.ColorDistanceSensor;

public class FakeColorDistance implements ColorDistanceSensor {
  private double m_threshold = 10.0;
  private boolean m_isRed = false;
  private double m_distance = 9999.0;

  public FakeColorDistance(double threshold_mm) {
    m_threshold = threshold_mm;
  }

  public FakeColorDistance(double threshold_mm, int dummy) {
    m_threshold = threshold_mm;
  }

  @Override
  public double getDistanceMillimeters() {
    return m_distance;
  }

  @Override
  public Color getColor() {
    if (m_distance < m_threshold) {
      if (m_isRed) {
        return Color.kRed;
      } else {
        return Color.kBlue;
      }
    } else {
      return Color.kBlack;
    }
  }
}
