package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Kicker extends RelEncoderSparkMax {

  public Kicker() {
    super(ShooterConstants.kKickerMotorConfig);
  }
}
