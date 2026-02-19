package frc.robot.util.motorUtil;

import frc.robot.Constants.Status;

public abstract class SparkIO extends MotorIO {

  public SparkIO(String name) {
    super(name);
  }

  @Override
  public void setPosition(double setpoint) {
    super.setPosition(setpoint);
  }

  @Override
  public void setSpeed(double speed) {
    super.setSpeed(speed);
  }

  @Override
  public void setPower(double power) {
    super.setPower(power);
  }

  public abstract double getPosition();

  @Override
  public abstract double getSpeed();

  @Override
  public abstract void setEncoder(double setpoint);

  public abstract double getPositionTolerance();

  public abstract double getSpeedTolerance();

  public abstract void runFFVelocity(double velocityRadPerSec);

  /** Returns the module velocity in rad/sec. */
  public abstract double getFFCharacterizationVelocity();

  // Runs specified voltage.
  public abstract void runCharacterization(double output);

  public abstract Status getStatus();
}
