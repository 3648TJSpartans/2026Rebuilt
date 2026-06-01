/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util.miscTunables;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableString extends Tunable<String> {

  /**
   * Create a new Tunable
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableString(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableString with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableString(String dashboardKey, String defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableString(String dashboardKey, String defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, String defaultValue) {
    SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
  }

  @Override
  protected String getDashboardValue(String key, String defaultValue) {
    return TUNING_MODE ? (String) SmartDashboard.getString(key, defaultValue) : defaultValue;
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, String value, String defaultValue) {
    Logger.recordOutput(
        key, TUNING_MODE ? SmartDashboard.getString(key, defaultValue) : defaultValue);
  }
}
