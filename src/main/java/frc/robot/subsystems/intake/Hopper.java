package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Hopper extends RelEncoderSparkMax {

  public Hopper() {
    super(IntakeConstants.hopperMotorConfig);
  }
}
