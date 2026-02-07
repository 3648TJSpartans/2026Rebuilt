package frc.robot.util.statusableUtils;

import frc.robot.Constants.Status;
import java.util.function.Supplier;

public class GenericStatusable implements Statusable {
  private final Supplier<Status> m_statusSupplier;
  private final String m_name;

  public GenericStatusable(Supplier<Status> statusSupplier, String name) {
    m_statusSupplier = statusSupplier;
    m_name = name;
  }

  public String getName() {
    return m_name;
  }

  public Status getStatus() {
    return m_statusSupplier.get();
  }
}
