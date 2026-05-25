package frc.robot.util.miscTunables;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

public class TunablePose3d extends Tunable<Pose3d> {
  private final TunableTranslation3d translation;
  private final TunableRotation3d rotation;

  public TunablePose3d(String dashboardKey) {
    super(dashboardKey);
    translation = new TunableTranslation3d(dashboardKey + "/translation");
    rotation = new TunableRotation3d(dashboardKey + "/rotation");
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose) {
    super(dashboardKey, defaultPose);
    translation =
        new TunableTranslation3d(dashboardKey + "/translation", defaultPose.getTranslation());
    rotation = new TunableRotation3d(dashboardKey + "/rotation", defaultPose.getRotation());
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose, Runnable update) {
    super(dashboardKey, defaultPose, update);
    translation =
        new TunableTranslation3d(dashboardKey + "/translation", defaultPose.getTranslation());
    rotation = new TunableRotation3d(dashboardKey + "/rotation", defaultPose.getRotation());
  }

  @Override
  protected void putDashboardValue(String key, Pose3d defaultValue) {
    translation.putDashboardValue(key, defaultValue.getTranslation());
    rotation.putDashboardValue(key, defaultValue.getRotation());
  }

  @Override
  protected Pose3d getDashboardValue(String key, Pose3d defaultValue) {
    return new Pose3d(
        translation.getDashboardValue(key, defaultValue.getTranslation()),
        rotation.getDashboardValue(key, defaultValue.getRotation()));
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Pose3d value, Pose3d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
