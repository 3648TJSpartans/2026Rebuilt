package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Shooter extends RelEncoderSparkMax{
  public Shooter(){
    super(ShooterConstants.kMotorConfig);
  }
}
