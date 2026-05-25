package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

public class TunableNumberAutoUpdater extends TunableNumber {

  private static final Set<TunableNumberAutoUpdater> UPDATERS = new HashSet<>();
  private final Runnable runnable;

  /**
   * Creates a new TunableNumberAutoUpdater. This is a tunable number that will automatically run a
   * given runnable whenever the value is changed from the dashboard. This is useful for things like
   * PID constants, where you want to be able to tune them on the fly without having to restart the
   * robot or manually check if they've changed.
   *
   * @param dashboardKey
   * @param defaultValue
   * @param runnahle
   */
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
