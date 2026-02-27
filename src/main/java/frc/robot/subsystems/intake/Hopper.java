package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends RelEncoderSparkMax {
  private boolean overrideJam;

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
    overrideJam = false;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/currentToSpeed")
  public double getCurrentToSpeed() {
    return Math.abs(getSpeed()) > IntakeConstants.speedThreshold.get()
            && Math.abs(getCurrent()) > IntakeConstants.currentThreshold.get()
        ? Math.abs(getCurrent() / getSpeed())
        : 0.0;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/jammed")
  public boolean jammed() {
    return getCurrentToSpeed() > IntakeConstants.jamThreshold.get();
  }

  public void overrideJam(boolean override) {
    overrideJam = override;
  }

  @AutoLogOutput(key = "Subsystems/Intake/Hopper/overrideJam")
  public boolean getOverrideJam() {
    return overrideJam;
  }

  @Override
  public void setPower(double power) {
    if (power != IntakeConstants.hopperUnjamPower.get()) {}
    super.setPower(power);
  }

  @Override
  public void stop() {
    super.stop();
  }

  public void run() {
    if (overrideJam) {
      setPower(IntakeConstants.hopperUnjamPower.get());
    }
    setPower(IntakeConstants.hopperSpeed.get());
  }
}
