package frc.robot.util.miscTunables;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import frc.robot.util.zoneCalc.Polygon;
import java.util.ArrayList;

public class TunablePolygon extends Tunable<Polygon> {
  private ArrayList<TunableTranslation2d> translation2ds;
  private TunableInteger corners;

  public TunablePolygon(String dashboardKey) {
    super(dashboardKey);
  }

  public TunablePolygon(String dashboardKey, Polygon defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunablePolygon(String dashboardKey, Polygon defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Polygon defaultValue) {
    // TODO Auto-generated method stub
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    final String dashboardKey = key;
    corners =
        new TunableInteger(
            dashboardKey + "/corners",
            defaultValue.getCorners().length,
            () -> updateTranslation2ds(dashboardKey, defaultValue));
    translation2ds = new ArrayList<>(corners.get());
    updateTranslation2ds(dashboardKey, defaultValue);
  }

  private void updateTranslation2ds(String key, Polygon defaultValue) {
    if (translation2ds.size() == corners.get()) {
      return;
    }
    if (translation2ds.size() < corners.get()) {
      for (int i = translation2ds.size(); i < corners.get(); i++) {

        TunableTranslation2d tunableTranslation2d =
            new TunableTranslation2d(
                key + "/corner" + i,
                i < defaultValue.getCorners().length
                    ? defaultValue.getCorners()[i]
                    : new Translation2d());
        translation2ds.add(tunableTranslation2d);
      }
    } else {
      for (int i = translation2ds.size() - 1; i >= corners.get(); i--) {
        translation2ds.remove(i);
      }
    }
  }

  @Override
  protected Polygon getDashboardValue(String key, Polygon defaultValue) {
    defaultValue.setCorners(
        translation2ds.stream().map(TunableTranslation2d::get).toArray(Translation2d[]::new));
    return defaultValue;
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Polygon value, Polygon defaultValue) {
    return;
  }
}
