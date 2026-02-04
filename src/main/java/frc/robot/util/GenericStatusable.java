package frc.robot.util;

import frc.robot.Constants.Status;
import java.util.function.Supplier;

public class GenericStatusable implements Statusable {
  private final Supplier<Boolean> m_isGood;
  private final String m_name;
private final Status m_falseStatus;

  public GenericStatusable(Supplier<Boolean> isGood, String name, Status falseStatus) {
    m_isGood = isGood;
    m_name = name;
    m_falseStatus = falseStatus;

  }
  public GenericStatusable(Supplier<Boolean> isGood, String name){
    this(isGood, name, Status.ERROR);
  }



  public String getName() {
    return m_name;
  }

  public Status getStatus() {
    return m_isGood.get() ? Status.OK : m_falseStatus;
  }
}
