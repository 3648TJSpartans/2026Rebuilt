/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util.miscTunables;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableRotation2d extends Tunable<Rotation2d> {

  /**
   * Create a new TunableRotation2d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableRotation2d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableRotation2d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableRotation2d(String dashboardKey, Rotation2d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableRotation2d(String dashboardKey, Rotation2d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Rotation2d defaultValue) {
    SmartDashboard.putNumber(
        key + "/radians", SmartDashboard.getNumber(key + "/radians", defaultValue.getRadians()));
  }

  @Override
  protected Rotation2d getDashboardValue(String key, Rotation2d defaultValue) {
    return TUNING_MODE
        ? new Rotation2d(
            SmartDashboard.getNumber(key + "/radians", defaultValue.getRadians()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Rotation2d value, Rotation2d defaultValue) {
    // System.out.println("Log Value run");
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
