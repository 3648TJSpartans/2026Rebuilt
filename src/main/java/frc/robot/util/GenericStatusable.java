package frc.robot.util;

import frc.robot.Constants.Status;
import java.util.function.Supplier;

public class GenericStatusable implements Statusable {
  private final Supplier<Boolean> m_isGood;
  private final String m_name;

  public GenericStatusable(Supplier<Boolean> isGood, String name) {
    m_isGood = isGood;
    m_name = name;
  }

  public String getName() {
    return m_name;
  }

  public Status getStatus() {
    return m_isGood.get() ? Status.OK : Status.ERROR;
  }
}
