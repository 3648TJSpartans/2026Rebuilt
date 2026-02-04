package frc.robot.subsystems.intake;

import frc.robot.Constants.Status;
import frc.robot.util.Statusable;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Hopper extends RelEncoderSparkMax {

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }
}
