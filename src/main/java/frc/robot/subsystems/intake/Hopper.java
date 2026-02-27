package frc.robot.subsystems.intake;

import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends RelEncoderSparkMax {

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }

  public double getCurrentToSpeed() {
    return Math.abs(getSpeed()) > 100 && Math.abs(getCurrent()) > .05
        ? Math.abs(getCurrent() / getSpeed())
        : 0.0;
  }

  @AutoLogOutput(key = "Subsystems/Hopper/jammed")
  public boolean jammed() {
    return getCurrentToSpeed() > IntakeConstants.jamThreshold.get();
  }
}
