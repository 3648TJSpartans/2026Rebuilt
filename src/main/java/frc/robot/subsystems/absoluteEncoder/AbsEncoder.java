package frc.robot.subsystems.absoluteEncoder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsEncoder extends SubsystemBase {
    private final AbsEncoderIO io;

    public AbsEncoder(AbsEncoderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // io.updateValues();
        Logger.recordOutput("absEncoder/position", io.getPosition());
    }

    public double getPosition() {
        return io.getPosition();
    }

    public void setTo(double setpoint) {
        Logger.recordOutput("absEncoder/setPos", setpoint);
        io.setTo(setpoint);
    }

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }
}
