/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableBoolean extends Supplier<Boolean> implements BooleanSupplier {

  private String key;
  private boolean defaultValue;
  private boolean lastHasChangedValue = defaultValue;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableBoolean(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public boolean getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
    if (TUNING_MODE) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  public boolean get() {
    Logger.recordOutput(
        key, TUNING_MODE ? SmartDashboard.getBoolean(key, defaultValue) : defaultValue);
    return TUNING_MODE ? SmartDashboard.getBoolean(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    boolean currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
