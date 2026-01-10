package frc.robot.subsystems.relativeEncoder;

/*
 * The IO file creates a skeleton for the third file with
 * most of the logic. If a subsystem could have two variants,
 * such as one with one motor and one with two, both could
 * run through the same IO structure.
 */

public interface RelEncoderIO {

    public default void setTo(double setpoint) {
    }

    public default boolean getLimitSwitch() {
        return false;
    }

    public default void stop() {
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetEncoder() {
    }

    public default void setSpeed(double speed) {
    }

    public default double getSpeed() {
        return 0;
    }

    public default void updateLimitSwitch() {
    }

    public default boolean atZero() {
        return true;
    }

    public default boolean getLimitReset() {
        return true;
    }
}
