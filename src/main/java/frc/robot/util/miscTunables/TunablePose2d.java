package frc.robot.util.miscTunables;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import org.littletonrobotics.junction.Logger;

public class TunablePose2d extends Tunable<Pose2d> {
  private TunableTranslation2d translation;
  private TunableRotation2d rotation;

  public TunablePose2d(String dashboardKey) {
    super(dashboardKey);
  }

  public TunablePose2d(String dashboardKey, Pose2d defaultPose) {
    super(dashboardKey, defaultPose);
  }

  public TunablePose2d(String dashboardKey, Pose2d defaultPose, Runnable update) {
    super(dashboardKey, defaultPose, update);
  }

  @Override
  protected void putDashboardValue(String key, Pose2d defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    translation = new TunableTranslation2d(key + "/translation", defaultValue.getTranslation());
    rotation = new TunableRotation2d(key + "/rotation", defaultValue.getRotation());
  }

  @Override
  protected Pose2d getDashboardValue(String key, Pose2d defaultValue) {
    return new Pose2d(
        translation.getDashboardValue(key + "/translation", defaultValue.getTranslation()),
        rotation.getDashboardValue(key + "/rotation", defaultValue.getRotation()));
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Pose2d value, Pose2d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
