package frc.robot.subsystems.simpleMotor;

/*
 * The IO file creates a skeleton for the third file with
 * most of the logic. If a subsystem could have two variants,
 * such as one with one motor and one with two, both could
 * run through the same IO structure.
 */

public interface SimpleMotorIO {

  public default void stop() {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }
}
