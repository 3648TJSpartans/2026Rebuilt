package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.statusableUtils.Statusable;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public abstract class MotorIO extends SubsystemBase implements Statusable {

  private static List<MotorIO> m_motorList = new ArrayList<MotorIO>();

  private final String name;
  private double m_speedSetpoint;
  private double m_positionSetpoint;
  private double m_powerSetpoint;

  public MotorIO(String loggingName) {
    this.name = loggingName;
    m_motorList.add(this);
  }

  public abstract double getPosition();

  public abstract double getSpeed();

  public void setPosition(double position) {
    m_positionSetpoint = position;
  }

  public void setSpeed(double speed) {
    m_speedSetpoint = speed;
  }

  public final boolean positionInTolerance() {
    return Math.abs(getPosition() - m_positionSetpoint) < getPositionTolerance();
  }

  public final boolean speedInTolerance() {
    return Math.abs(getSpeed() - m_speedSetpoint) < getSpeedTolerance();
  }

  public void setPower(double power) {
    m_powerSetpoint = power;
  }
  ;

  public void updateValues() {
    Logger.recordOutput(name + "/getPosition", getPosition());
    Logger.recordOutput(name + "/getSpeed", getSpeed());
    Logger.recordOutput(name + "/setPose", m_positionSetpoint);
    Logger.recordOutput(name + "/setSpeed", m_speedSetpoint);
    Logger.recordOutput(name + "/poseInTolerance", positionInTolerance());
    Logger.recordOutput(name + "/speedInTolerance", speedInTolerance());
    Logger.recordOutput(name + "/setPower", m_powerSetpoint);
  }

  public abstract void setEncoder(double setpoint);

  public final void resetEncoder() {
    setEncoder(0.0);
  }

  public void stop() {
    setPower(0.0);
    m_speedSetpoint = 0.0;
  }

  public String getName() {
    return name;
  }

  public abstract double getPositionTolerance();

  public abstract double getSpeedTolerance();

  @Override
  public void periodic() {
    updateValues();
  }

  public void configureMotor() {}

  public static final void reconfigureMotors() {
    for (MotorIO motor : m_motorList) {
      motor.configureMotor();
    }
  }
}
