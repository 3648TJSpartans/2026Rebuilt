package frc.robot.util.statusableUtils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Status;
import java.util.function.Supplier;

public class GenericStatusable implements Statusable {
  private final Supplier<Status> m_statusSupplier;
  private final String m_name;
  private Status m_cachedStatus;
  private double m_lastUpdateTime = 0;
  private int m_fetchDelay;

  public GenericStatusable(Supplier<Status> statusSupplier, String name, int fetchDelay) {
    m_statusSupplier = statusSupplier;
    m_name = name;
    m_fetchDelay = fetchDelay;
  }

  public GenericStatusable(Supplier<Status> statusSupplier, String name) {
    this(statusSupplier, name, 0);
  }

  public String getName() {
    return m_name;
  }

  public Status getStatus() {
    if (Timer.getFPGATimestamp() > m_lastUpdateTime + m_fetchDelay) {
      m_cachedStatus = m_statusSupplier.get();
      m_lastUpdateTime = Timer.getFPGATimestamp();
      return m_cachedStatus;
    }
    return m_cachedStatus;
  }
}
