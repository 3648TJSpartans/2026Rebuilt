package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends RelEncoderSparkMax {

  private double setPower;

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
    setPower = 0;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/currentToSpeed")
  public double getCurrentToSpeed() {
    return Math.abs(getSpeed()) > 100 && Math.abs(getCurrent()) > .05
        ? Math.abs(getCurrent() / getSpeed())
        : 0.0;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/jammed")
  public boolean jammed() {
    return getCurrentToSpeed() > IntakeConstants.jamThreshold.get();
  }

  @Override
  public void setPower(double power) {
    if (power != IntakeConstants.hopperUnjamPower.get()) {
      setPower = power;
    }
    super.setPower(power);
  }

  @Override
  public void stop() {
    setPower = 0;
    super.stop();
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/nonJamPower")
  public double getPower() {
    return setPower;
  }
}
