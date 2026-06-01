package frc.robot.util;

import static frc.robot.util.TuningUpdater.*;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class Tunable<E> implements Supplier<E> {

  private String key;
  private E defaultValue;
  private E lastHasChangedValue = defaultValue;
  private final Optional<Runnable> update;
  private final Optional<Consumer<E>> updateConsumer;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public Tunable(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.update = Optional.empty();
    this.updateConsumer = Optional.empty();
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public Tunable(String dashboardKey, E defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  public Tunable(String dashboardKey, E defaultValue, Runnable update) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    setDefault(defaultValue);
    this.update = Optional.of(update);
    this.updateConsumer = Optional.empty();
    TuningUpdater.addAutoUpdater(this);
  }

  public Tunable(String dashboardKey, E defaultValue, Consumer<E> updateConsumer) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    setDefault(defaultValue);
    this.update = Optional.empty();
    this.updateConsumer = Optional.of(updateConsumer);
    TuningUpdater.addAutoUpdater(this);
  }

  public Tunable(String dashboardKey, E defaultValue, Runnable update, Consumer<E> updateConsumer) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    setDefault(defaultValue);
    this.update = Optional.of(update);
    this.updateConsumer = Optional.of(updateConsumer);
    TuningUpdater.addAutoUpdater(this);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public final E getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public final void setDefault(E defaultValue) {
    this.defaultValue = defaultValue;
    if (TUNING_MODE) {
      // This makes sure the data is on NetworkTables but will not change it
      putDashboardValue(key, defaultValue);
    }
  }

  @Override
  public final E get() {
    logValue(key, TUNING_MODE, lastHasChangedValue, defaultValue);
    return TUNING_MODE ? getDashboardValue(key, defaultValue) : defaultValue;
  }

  public final boolean hasChanged() {
    E currentValue = get();
    if (!currentValue.equals(lastHasChangedValue)) {
      lastHasChangedValue = currentValue;
      return true;
    }
    return false;
  }

  public final void update() {
    if (hasChanged()) {
      if (update.isPresent()) {
        update.get().run();
      }
      if (updateConsumer.isPresent()) {
        updateConsumer.get().accept(lastHasChangedValue);
      }
    }
  }

  protected abstract void putDashboardValue(String key, E defaultValue);

  protected abstract E getDashboardValue(String key, E defaultValue);

  protected abstract void logValue(String key, boolean TUNING_MODE, E value, E defaultValue);
}
