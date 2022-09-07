package frc.robot.simulation;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.subsystems.indexer.BallColorDetector;

public class BallColorDetectorSim {
  private final SimDeviceSim m_simDeviceSim;
  private final SimInt m_simR;
  private final SimInt m_simG;
  private final SimInt m_simB;
  private final SimDouble m_simDistance;

  public BallColorDetectorSim(BallColorDetector detector) {
    m_simDeviceSim = new SimDeviceSim("Ball Color Detector", detector.getIndex());
    m_simR = m_simDeviceSim.getInt("Red");
    m_simG = m_simDeviceSim.getInt("Green");
    m_simB = m_simDeviceSim.getInt("Blue");
    m_simDistance = m_simDeviceSim.getDouble("Distance");
  }

  public void setColor(int r, int g, int b) {
    m_simR.set(r);
    m_simG.set(g);
    m_simB.set(b);
  }

  public void setDistance(double distanceMillimeters) {
    m_simDistance.set(distanceMillimeters);
  }
}
