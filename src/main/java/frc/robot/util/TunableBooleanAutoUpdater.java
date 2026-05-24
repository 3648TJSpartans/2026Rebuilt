package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

public class TunableBooleanAutoUpdater extends TunableBoolean {

  private static final Set<TunableBooleanAutoUpdater> UPDATERS = new HashSet<>();
  private final Runnable runnable;

  public TunableBooleanAutoUpdater(String dashboardKey, boolean defaultValue, Runnable runnahle) {
    super(dashboardKey, defaultValue);
    this.runnable = runnahle;
    UPDATERS.add(this);
  }

  public static void update() {
    for (TunableBooleanAutoUpdater updater : TunableBooleanAutoUpdater.UPDATERS) {
      if (updater.hasChanged()) {
        updater.runnable.run();
      }
    }
  }
}
