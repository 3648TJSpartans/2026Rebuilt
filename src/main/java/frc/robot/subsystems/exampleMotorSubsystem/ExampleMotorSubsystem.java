package frc.robot.subsystems.exampleMotorSubsystem;

import javax.tools.Diagnostic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Diagnostics;
import frc.robot.util.motorUtil.MotorIO;

public class ExampleMotorSubsystem extends SubsystemBase {
    private final MotorIO m_motor1;
    private final MotorIO m_motor2;

    public ExampleMotorSubsystem() {
        m_motor1 = ExampleMotorSubsystemConstants.motor1;
        m_motor2 = ExampleMotorSubsystemConstants.motor2;
    }

    public void setPower(double power) {
        m_motor1.setPower(power);
        m_motor2.setPower(power);
    }

    public void stop() {
        m_motor1.stop();
        m_motor2.stop();
    }
}
