package frc.robot.util.miscTunables;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import org.littletonrobotics.junction.Logger;

public class TunablePose3d extends Tunable<Pose3d> {
  private TunableTranslation3d translation;
  private TunableRotation3d rotation;

  public TunablePose3d(String dashboardKey) {
    super(dashboardKey);
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose) {
    super(dashboardKey, defaultPose);
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose, Runnable update) {
    super(dashboardKey, defaultPose, update);
  }

  @Override
  protected void putDashboardValue(String key, Pose3d defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    translation = new TunableTranslation3d(key + "/translation", defaultValue.getTranslation());
    rotation = new TunableRotation3d(key + "/rotation", defaultValue.getRotation());
  }

  @Override
  protected Pose3d getDashboardValue(String key, Pose3d defaultValue) {
    return new Pose3d(
        translation.getDashboardValue(key + "/translation", defaultValue.getTranslation()),
        rotation.getDashboardValue(key + "/rotation", defaultValue.getRotation()));
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Pose3d value, Pose3d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
