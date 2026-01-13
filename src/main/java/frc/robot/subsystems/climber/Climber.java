package frc.robot.subsystems.climber;

import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Climber extends RelEncoderSparkMax {

  public Climber() {
    super(ClimberConstants.climbMotorConfig);
  }
}
