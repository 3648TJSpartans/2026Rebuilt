/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util.miscTunables;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableRotation3d extends Tunable<Rotation3d> {

  /**
   * Create a new TunableRotation3d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableRotation3d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableRotation3d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableRotation3d(String dashboardKey, Rotation3d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableRotation3d(String dashboardKey, Rotation3d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Rotation3d defaultValue) {
    SmartDashboard.putNumber(
        key + "/pitch", SmartDashboard.getNumber(key + "/pitch", defaultValue.getX()));
    SmartDashboard.putNumber(
        key + "/yaw", SmartDashboard.getNumber(key + "/yaw", defaultValue.getY()));
    SmartDashboard.putNumber(
        key + "/roll", SmartDashboard.getNumber(key + "/roll", defaultValue.getZ()));
  }

  @Override
  protected Rotation3d getDashboardValue(String key, Rotation3d defaultValue) {
    return TUNING_MODE
        ? new Rotation3d(
            SmartDashboard.getNumber(key + "/pitch", defaultValue.getX()),
            SmartDashboard.getNumber(key + "/yaw", defaultValue.getY()),
            SmartDashboard.getNumber(key + "/roll", defaultValue.getZ()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Rotation3d value, Rotation3d defaultValue) {
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
