package frc.robot.util.statusableUtils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Status;

public class StatusableDigitalInput extends DigitalInput implements Statusable {
  private String name;
  private boolean hasChanged = false;
  private final boolean initialValue;

  /*
   * For updates to work properly, get needs to be run periodically.
   */
  public StatusableDigitalInput(int channel, String name) {
    super(channel);
    this.name = name;
    this.initialValue = super.get();
  }

  @Override
  public boolean get() {
    boolean currentValue = super.get();
    if (currentValue != initialValue) {
      hasChanged = true;
    }
    return currentValue;
  }

  @Override
  public Status getStatus() {

    return hasChanged ? Status.OK : Status.WARNING;
  }

  @Override
  public String getName() {
    return name;
  }
}
