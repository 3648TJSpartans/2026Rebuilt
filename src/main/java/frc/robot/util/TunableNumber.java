/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.util;

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber extends Tunable<Double> implements DoubleSupplier {

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public double getAsDouble() {
    return get();
  }

  @Override
  protected void putDashboardValue(String key, Double defaultValue) {
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
  }

  @Override
  protected Double getDashboardValue(String key, Double defaultValue) {
    return TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Double value, Double defaultValue) {
    Logger.recordOutput(
        key, TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue);
  }
}
