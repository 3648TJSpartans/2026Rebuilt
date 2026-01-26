package frc.robot.subsystems.shooter;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ShooterConstants {
  public static MotorConfig kMotorConfigLead =
      new MotorConfig("Subsystems/Shooter/MotorIO").motorCan(13).Ks(0).Kv(0).speedTolerance(0.0);
public static MotorConfig kMotorConfigFollow =
      new MotorConfig("Subsystems/Shooter/MotorIO").motorCan(14).Ks(0).Kv(0).speedTolerance(0.0).isInverted(true);
  /*Used to convert from RPM to m/s the ball is shot. RPM*s/m
   * Example: If revving the shooter to 5000 rpm results in a ball velocity of 15 m/s, then the factor will be 5000/15 = 333.33 RPM*s/m
   */
  
  public static final TunableNumber kShooterVelocityFactor =
      new TunableNumber("Subsystems/Shooter/VelocityFactor", 400);
}
