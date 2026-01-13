package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.AbsEncoderSparkMax;

public class Pivot extends AbsEncoderSparkMax {

  public Pivot() {
    super(ShooterConstants.pivotMotorConfig);
  }
}
