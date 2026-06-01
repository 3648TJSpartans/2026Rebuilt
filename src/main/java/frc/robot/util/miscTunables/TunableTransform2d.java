package frc.robot.util.miscTunables;

import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import org.littletonrobotics.junction.Logger;

public class TunableTransform2d extends Tunable<Transform2d> {
  private TunableTranslation2d translation;
  private TunableRotation2d rotation;

  public TunableTransform2d(String dashboardKey) {
    super(dashboardKey);
  }

  public TunableTransform2d(String dashboardKey, Transform2d defaultTransform) {
    super(dashboardKey, defaultTransform);
  }

  public TunableTransform2d(String dashboardKey, Transform2d defaultTransform, Runnable update) {
    super(dashboardKey, defaultTransform, update);
  }

  @Override
  protected void putDashboardValue(String key, Transform2d defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    translation = new TunableTranslation2d(key + "/translation", defaultValue.getTranslation());
    rotation = new TunableRotation2d(key + "/rotation", defaultValue.getRotation());
  }

  @Override
  protected Transform2d getDashboardValue(String key, Transform2d defaultValue) {
    return new Transform2d(
        translation.getDashboardValue(key + "/translation", defaultValue.getTranslation()),
        rotation.getDashboardValue(key + "/rotation", defaultValue.getRotation()));
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Transform2d value, Transform2d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
