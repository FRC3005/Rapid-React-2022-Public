package frc.robot.subsystems.indexer;

import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.vendor.sensor.ColorDistanceSensor;

public class BallColorDetector implements ColorDistanceSensor {
  // Global thread for multiple color sensors
  private static DualColorSensorThread m_colorSensor = new DualColorSensorThread(Port.kMXP);
  private double m_thresholdMillimeters;
  private boolean m_state = false;
  private double m_hysterisisMillimeters = 5.0;
  private int m_sensorIdx;

  // Sim functions
  private final SimDevice m_simDevice;
  private final SimInt m_simR;
  private final SimInt m_simG;
  private final SimInt m_simB;
  private final SimDouble m_simDistance;

  public BallColorDetector(double threshold_mm, int sensorIdx) {
    m_thresholdMillimeters = threshold_mm;
    m_sensorIdx = sensorIdx;
    m_simDevice = SimDevice.create("Ball Color Detector", sensorIdx);

    if (m_simDevice != null) {
      m_simR = m_simDevice.createInt("Red", Direction.kBidir, 0);
      m_simG = m_simDevice.createInt("Green", Direction.kBidir, 0);
      m_simB = m_simDevice.createInt("Blue", Direction.kBidir, 0);
      m_simDistance = m_simDevice.createDouble("Distance", Direction.kBidir, 0.0);
    } else {
      m_simR = null;
      m_simG = null;
      m_simB = null;
      m_simDistance = null;
    }
  }

  /**
   * Get the color either Color.kRed, or Color.kBlue if the game piece is present.
   *
   * @return eithr Color.kRed or Color.kBlue
   */
  @Override
  public Color getColor() {
    if (!isPresent()) {
      return Color.kBlack;
    }

    RawColor color;

    if (m_simDevice != null) {
      color = new RawColor(m_simR.get(), m_simG.get(), m_simB.get(), 0);
    } else {
      color = m_colorSensor.getRawColor(m_sensorIdx);
    }

    // Add extra gain to 'equal out' the spectral response. See Figure 1. of the APDS-9151 datasheet
    double blue = (double) color.blue * 1.666666;
    double red = (double) color.red * 1.11111111;

    if (red > blue) {
      return Color.kRed;
    } else {
      return Color.kBlue;
    }
  }

  public RawColor getRawColor() {
    return m_colorSensor.getRawColor(m_sensorIdx);
  }

  /**
   * Return the distance of the game piece if it is present.
   *
   * @return distance of the game piece if present
   */
  @Override
  public double getDistanceMillimeters() {
    if (m_simDevice != null) {
      return m_simDistance.get();
    }

    return m_colorSensor.getProximityMillimeters(m_sensorIdx);
  }

  /**
   * Return if the game piece is in range
   *
   * @return true if game piece is in range
   */
  public boolean isPresent() {
    double distance = getDistanceMillimeters();
    if (distance < m_thresholdMillimeters) {
      m_state = true;
    } else if (distance > m_thresholdMillimeters + m_hysterisisMillimeters) {
      m_state = false;
    }
    return m_state;
  }

  /**
   * Return the sensor index in the color sensor thread. This is the value sent to the constructor.
   *
   * @return
   */
  public int getIndex() {
    return m_sensorIdx;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ColorDistanceSensor.super.initSendable(builder);
    builder.addBooleanProperty("State", () -> m_state, null);
    builder.addDoubleProperty("Raw Red", () -> getRawColor().red * 1.1111111, null);
    builder.addDoubleProperty("Raw Green", () -> getRawColor().green, null);
    builder.addDoubleProperty("Raw Blue", () -> getRawColor().blue * 1.666666, null);
    builder.addDoubleProperty(
        "Threshold mm", () -> m_thresholdMillimeters, (val) -> m_thresholdMillimeters = val);
  }
}
