package frc.robot.util.motorUtil;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MultiMotorSubsystem extends SubsystemBase {
    private final MotorIO[] m_motors;

    public MultiMotorSubsystem(MotorIO... motors) {
        m_motors = motors;
    }

    public void setPosition(double position) {
        for (MotorIO motor : m_motors) {
            motor.setPosition(position);
        }
    }

    public void setSpeed(double speed) {
        for (MotorIO motor : m_motors) {
            motor.setSpeed(speed);
        }
    }

    public void setPower(double power) {
        for (MotorIO motor : m_motors) {
            motor.setPower(power);
        }
    }

    public void setEncoder(double setpoint) {
        for (MotorIO motor : m_motors) {
            motor.setEncoder(setpoint);
        }
    }

    public final void resetEncoder() {
        for (MotorIO motor : m_motors) {
            motor.resetEncoder();
        }
    }

    public final void stop() {
        for (MotorIO motor : m_motors) {
            motor.stop();
        }
    }

}
