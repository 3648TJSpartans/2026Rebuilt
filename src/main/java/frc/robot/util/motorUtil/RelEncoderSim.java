package frc.robot.util.motorUtil;

import frc.robot.Constants.Status;
import frc.robot.util.TunableNumber;

public class RelEncoderSim extends RelEncoderIO {
  private double m_position = 0.0;
  private double m_speed = 0.0;
  private double m_power = 0.0;
  private final TunableNumber kV;

  public RelEncoderSim(String name, TunableNumber kV) {
    super(name);
    this.kV = kV;
  }

  @Override
  public void periodic() {
    m_speed = m_power * kV.get();
    m_position += m_speed * 0.02;
    super.periodic();
  }

  @Override
  public void setPosition(double setpoint) {
    super.setPosition(setpoint);
    m_position = setpoint;
  }

  @Override
  public void setSpeed(double speed) {
    super.setSpeed(speed);
    m_speed = speed;
  }

  @Override
  public void setPower(double power) {
    super.setPower(power);
    m_power = power;
  }

  @Override
  public double getPosition() {
    return m_position;
  }

  @Override
  public double getSpeed() {
    return m_speed;
  }

  @Override
  public void setEncoder(double setpoint) {
    m_position = setpoint;
  }

  public double getPositionTolerance() {
    return 0.0;
  }

  public double getSpeedTolerance() {
    return 0.0;
  }

  @Override
  public void configureMotor() {}

  public void runFFVelocity(double velocityRadPerSec) {
    setSpeed(velocityRadPerSec);
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return getSpeed();
  }

  // Runs specified voltage.
  public void runCharacterization(double output) {
    super.setPower(output);
    m_power = output;
  }

  public Status getStatus() {
    return Status.UNKNOWN;
  }
}
