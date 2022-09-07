package frc.robot.subsystems.indexer;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Notifier;
import frc.lib.vendor.sensor.I2CMux;
import frc.lib.vendor.sensor.TCA9548A;
import java.util.ArrayList;
import java.util.List;

public class DualColorSensorThread implements AutoCloseable {
  private final I2CMux m_mux = new TCA9548A();
  private final Notifier m_notifier;
  private final int kNumSensors = 2;
  private final double kSamplePeriod = 0.02;
  private final int[] kSensorMuxIndex = {1, 2};
  private volatile SensorData[] m_sensorData = new SensorData[kNumSensors];
  private final List<ColorSensorV3> m_sensors = new ArrayList<>();

  private class SensorData {
    public RawColor color;
    public double distance;
  }

  public DualColorSensorThread(Port port) {
    m_sensorData[0] = new SensorData();
    m_sensorData[1] = new SensorData();

    assert kNumSensors < m_mux.availableBuses();
    for (var idx : kSensorMuxIndex) {
      assert idx < m_mux.availableBuses();
    }

    for (int i = 0; i < kNumSensors; i++) {
      m_mux.setEnabledBuses(kSensorMuxIndex[i]);
      var sensor = new ColorSensorV3(port);
      sensor.configureProximitySensor(
          ProximitySensorResolution.kProxRes8bit, ProximitySensorMeasurementRate.kProxRate12ms);
      sensor.configureColorSensor(
          ColorSensorResolution.kColorSensorRes13bit,
          ColorSensorMeasurementRate.kColorRate25ms,
          GainFactor.kGain9x);
      m_sensorData[i].color = new RawColor(0, 0, 0, 0);
      m_sensorData[i].distance = 0.0;
      m_sensors.add(sensor);
    }

    // Run this in a thread since I2C can be shady, and many transactions can be expensive
    m_notifier =
        new Notifier(
            () -> {
              for (int i = 0; i < kNumSensors; i++) {
                m_mux.setEnabledBuses(kSensorMuxIndex[i]);
                var sensor = m_sensors.get(i);
                m_sensorData[i].color = sensor.getRawColor();
                m_sensorData[i].distance = sensor.getProximity();
              }
            });

    m_notifier.startPeriodic(kSamplePeriod);
  }

  public RawColor getRawColor(int sensorIndex) {
    assert sensorIndex < kNumSensors;
    return m_sensorData[sensorIndex].color;
  }

  public double getProximityMillimeters(int sensorIndex) {
    assert sensorIndex < kNumSensors;
    // TODO: Convert to mm
    return m_sensorData[sensorIndex].distance;
  }

  @Override
  public void close() {
    m_notifier.stop();
    m_notifier.close();
  }
}
