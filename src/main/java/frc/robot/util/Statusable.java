package frc.robot.util;

import frc.robot.Constants.Status;

public interface Statusable {
  public abstract Status getStatus();

  public abstract String getName();
}
