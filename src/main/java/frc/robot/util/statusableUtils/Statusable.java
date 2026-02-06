package frc.robot.util.statusableUtils;

import frc.robot.Constants.Status;

public interface Statusable {
  public abstract Status getStatus();

  public abstract String getName();
}
