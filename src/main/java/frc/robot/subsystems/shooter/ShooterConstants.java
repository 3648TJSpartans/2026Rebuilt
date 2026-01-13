package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.MotorConfig;

public class ShooterConstants {
  public static MotorConfig kMotorConfig =
      new MotorConfig("Subsystems/Shooter/MotorIO").motorCan(13).Ks(0).Kv(0).speedTolerance(0.0);
}
