package frc.robot.subsystems.theClaw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.SparkIO;
import frc.robot.util.statusableUtils.Statusable;
import frc.robot.util.statusableUtils.StatusableDigitalInput;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TheClaw extends SubsystemBase implements Statusable {
  private final StatusableDigitalInput bottomSwitch;
  private final SparkIO m_motor;
  private boolean isHomed;

  public TheClaw(SparkIO motor) {
    m_motor = motor;
    isHomed = Constants.currentMode == Mode.SIM ? true : false;
    bottomSwitch =
        new StatusableDigitalInput(TheClawstants.bottomSwitchPort, "Subsystems/Claw/limitSwitch");
  }

  public void setPosition(double position) {
    if (isHomed) {
      position =
          MathUtil.clamp(
              position, TheClawstants.minPosition.get(), TheClawstants.maxPosition.get());
      if (bottomSwitch.get()) {
        if (position < m_motor.getPosition()) {
          m_motor.setPower(0.0);
          return;
        }
      }
      m_motor.setPosition(position);
    }
  }

  @AutoLogOutput(key = "Subsystems/Claw/homed")
  public boolean isHomed() {
    return isHomed;
  }

  public void setZero() {
    isHomed = true;
    m_motor.setEncoder(TheClawstants.minPosition.get());
  }

  public Translation3d getTranslation() {
    return new Translation3d(0, 0, m_motor.getPosition() * TheClawstants.encoderPositionFactor);
  }

  public void setHeight(double height) {
    setPosition(height / TheClawstants.encoderPositionFactor);
  }

  @Override
  public void periodic() {
    if (bottomSwitch.get()) {
      setZero();
    }

    m_motor.periodic();
    updateInputs();
  }

  public void updateInputs() {
    Logger.recordOutput("Subsystems/Claw/position", getTranslation());
    Logger.recordOutput("Subsystems/Claw/limitSwitch", bottomSwitch.get());
  }

  @Override
  public Status getStatus() {
    if (m_motor.getStatus() != Status.OK) {
      return m_motor.getStatus();
    }
    return bottomSwitch.getStatus();
  }

  public void setPower(double power) {
    if (bottomSwitch.get()) {
      if (power > 0.0) {
        m_motor.stop();
        return;
      }
    }
    m_motor.setPower(power);
  }

  public void stop() {
    m_motor.setPower(0.0);
  }

  public SparkIO getMotor() {
    return m_motor;
  }
}
