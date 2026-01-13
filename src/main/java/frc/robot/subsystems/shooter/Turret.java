package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Turret extends RelEncoderSparkMax {

  public Turret() {
    super(ShooterConstants.turretMotorConfig);
  }
}
