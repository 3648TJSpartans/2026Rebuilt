package frc.robot.subsystems.absoluteEncoder;

public interface AbsEncoderIO {
    public default double getPosition() {
        return 0;
    }

    public default void setTo(double setpoint) {
    }

    public default void stop() {
    }

    public default void updateValues() {

    }

    public default void setSpeed(double speed) {

    }
}
