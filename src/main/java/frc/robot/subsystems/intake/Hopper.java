package frc.robot.subsystems.intake;

import frc.robot.Constants.Status;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

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

  public boolean jammed() {
    return getCurrentToSpeed() > ClimberConstants.currentToSpeedThreshold.get();
  }
}
