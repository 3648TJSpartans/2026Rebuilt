package frc.robot.util.statusableUtils;

import frc.robot.Constants.Status;

public interface Statusable {
  public default Status getStatus() {
    return Status.UNKNOWN;
  }

  public abstract String getName();
}
