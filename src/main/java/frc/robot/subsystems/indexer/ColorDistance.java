package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.util.Color;

public class ColorDistance {
  public Color color;
  public double distance;

  public ColorDistance() {
    color = Color.kBlack;
    distance = 0.0;
  }

  public ColorDistance(Color color, double distance_mm) {
    this.color = color;
    this.distance = distance_mm;
  }

  public static ColorDistance makeColorDistance(Color color, double distance_mm) {
    return new ColorDistance(color, distance_mm);
  }
}
