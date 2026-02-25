package frc.robot.util.statusableUtils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Status;
import org.littletonrobotics.junction.Logger;

public class StatusableDigitalInput extends DigitalInput implements Statusable {
  private String name;
  private boolean hasChanged = false;
  private boolean lastValue;
  private boolean currentValue;
  private boolean switchedTrue;
  private boolean switchedFalse;
  private final boolean initialValue;

  /*
   * For updates to work properly, get needs to be run periodically.
   */
  public StatusableDigitalInput(int channel, String name) {
    super(channel);
    this.name = name;
    this.initialValue = super.get();
    this.lastValue = initialValue;
    this.currentValue = lastValue;
    switchedTrue = false;
    switchedFalse = false;
  }

  public void updateValues() {
    lastValue = currentValue;
    currentValue = super.get();
    if (currentValue != initialValue) {
      hasChanged = true;
    }
    if (currentValue != lastValue) {
      if (currentValue) {
        switchedTrue = true;
      } else {
        switchedFalse = true;
      }
    } else {
      switchedTrue = false;
      switchedFalse = false;
    }
    Logger.recordOutput(name + "/value", currentValue);
    Logger.recordOutput(name + "/switchedTrue", switchedTrue);
    Logger.recordOutput(name + "/switchedFalse", switchedFalse);
  }

  @Override
  public boolean get() {
    return currentValue;
  }

  public boolean switchedTrue() {
    return switchedTrue;
  }

  public boolean switchedFalse() {
    return switchedFalse;
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
