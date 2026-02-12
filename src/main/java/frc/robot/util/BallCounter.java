package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BallCounter extends SubsystemBase {
  private final Queue<Double> m_velocityHistory;
  private final Queue<Double> m_timeHistory;
  private final DoubleSupplier m_velocitySupplier;
  private int m_ballCount;
  private double m_c2;

  private static final int countSize = 10;

  public BallCounter(DoubleSupplier velocitySupplier) {
    m_velocityHistory = new ArrayBlockingQueue<>(countSize);
    m_timeHistory = new ArrayBlockingQueue<>(countSize);
    m_velocitySupplier = velocitySupplier;
    m_ballCount = 0;
  }

  @Override
  public void periodic() {
    updateInputs();
    updateC2();
    Logger.recordOutput("Utils/BallCounter/getC2", m_c2);
    Logger.recordOutput(
        "Utils/BallCounter/velocityArray",
        m_velocityHistory.stream().mapToDouble((Double value) -> value).toArray());
    Logger.recordOutput(
        "Utils/BallCounter/timeArray",
        m_timeHistory.stream().mapToDouble((Double value) -> value).toArray());
  }

  public void updateInputs() {
    m_velocityHistory.offer(
        m_velocitySupplier.getAsDouble()); // Replace with actual velocity from supplier
    m_timeHistory.offer(System.currentTimeMillis() / 1000.0); // Current
  }

  private void updateC2() {
    if (m_velocityHistory.size() == countSize) {
      double num = 0;
      for (double velocity : m_velocityHistory) {
        num += velocity * velocity;
      }
      double midTime = (Double) m_timeHistory.toArray()[countSize / 2];
      double den = 0;
      for (double time : m_timeHistory) {
        double delta = time - midTime;
        den += delta * delta * delta * delta;
      }
      clearQueues();
      m_c2 = num / den;
    }
  }

  private void clearQueues() {
    m_velocityHistory.clear();
    m_timeHistory.clear();
  }

  @AutoLogOutput(key = "Utils/BallCounter/getBallCount")
  public double getBallCount() {
    return m_ballCount;
  }
}
