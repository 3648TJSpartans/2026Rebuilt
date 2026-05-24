package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

public class TunableNumberAutoUpdater extends TunableNumber {

  private static final Set<TunableNumberAutoUpdater> UPDATERS = new HashSet<>();
  private final Runnable runnable;

  public TunableNumberAutoUpdater(String dashboardKey, double defaultValue, Runnable runnahle) {
    super(dashboardKey, defaultValue);
    this.runnable = runnahle;
    UPDATERS.add(this);
  }

  public static void update() {
    for (TunableNumberAutoUpdater updater : TunableNumberAutoUpdater.UPDATERS) {
      if (updater.hasChanged()) {
        updater.runnable.run();
      }
    }
  }
}
