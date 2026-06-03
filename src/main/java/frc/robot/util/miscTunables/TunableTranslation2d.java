/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util.miscTunables;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableTranslation2d extends Tunable<Translation2d> {

  /**
   * Create a new TunableTranslation2d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableTranslation2d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableTranslation2d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableTranslation2d(String dashboardKey, Translation2d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableTranslation2d(String dashboardKey, Translation2d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Translation2d defaultValue) {
    SmartDashboard.putNumber(key + "/x", SmartDashboard.getNumber(key + "/x", defaultValue.getX()));
    SmartDashboard.putNumber(key + "/y", SmartDashboard.getNumber(key + "/y", defaultValue.getY()));
  }

  @Override
  protected Translation2d getDashboardValue(String key, Translation2d defaultValue) {
    return TUNING_MODE
        ? new Translation2d(
            SmartDashboard.getNumber(key + "/x", defaultValue.getX()),
            SmartDashboard.getNumber(key + "/y", defaultValue.getY()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Translation2d value, Translation2d defaultValue) {
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
