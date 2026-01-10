package frc.robot.subsystems.relativeEncoder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This subsystem controls a single motor with a relative motor, which means
 * the robot knows how many times the robot has spun since it turned on. This
 * means it can track a motor that makes revolutons over 360 degrees, unlike
 * absolute encoders, but it also starts counting at 0 no matter where the 
 * motor is when turned on.
 */

/*
 * Subsystems are split into a main file, an IO, and a third file which houses
 * most of the logic. This file, the main file, receives an IO, and uses it
 * to structure the subsystem.
 */

public class RelEncoder extends SubsystemBase {

  private final RelEncoderIO io;

  public RelEncoder(RelEncoderIO io) {
    this.io = io;
  }

  /* Periodic, which runs constantly, is used here to log important values. */

  @Override
  public void periodic() {

    Logger.recordOutput("relEncoder/EncoderValue", io.getPosition());
    Logger.recordOutput("relEncoder/speed", io.getSpeed());

    io.updateLimitSwitch();
  }

  /* The remaining methods are all taken from the IO. */

  public boolean getLimitSwitch() {
    return io.getLimitSwitch();
  }

  public void setTo(double setpoint) {
    io.setTo(setpoint);
  }

  public void stop() {
    io.stop();
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public boolean atZero() {
    return io.atZero();
  }

  public boolean getLimitReset() {
    return io.getLimitReset();
  }
}
