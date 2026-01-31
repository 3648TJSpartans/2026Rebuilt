package frc.robot.subsystems.shooter;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ShooterConstants {
  public static MotorConfig kLeaderMotorConfig =
      new MotorConfig("Subsystems/Shooter/LeaderMotorIO")
          .motorCan(10)
          .Ks(1.06085)
          .Kv(0.00206)
          .speedTolerance(0.0);
  public static MotorConfig kFollowerMotorConfig =
      new MotorConfig("Subsystems/Shooter/FollowerMotorIO")
          .motorCan(11)
          .Ks(0)
          .Kv(0)
          .speedTolerance(0.0)
          .follow(12)
          .isInverted(true);

  public static MotorConfig kKickerMotorConfig =
      new MotorConfig("Subsystems/Shooter/KickerMotorIO")
          .motorCan(13)
          .Ks(0)
          .Kv(0)
          .speedTolerance(0.0);

  public static int kickerIRSensorChannel = 3;

  /*Used to convert from RPM to m/s the ball is shot. RPM*s/m
   * Example: If revving the shooter to 5000 rpm results in a ball velocity of 15 m/s, then the factor will be 5000/15 = 333.33 RPM*s/m
   */
  public static final TunableNumber kShooterVelocityFactor =
      new TunableNumber("Subsystems/Shooter/VelocityFactor", 400);
  public static final TunableNumber kickerSpeed =
      new TunableNumber("Subsystems/Shooter/kickerSpeed", 0.1);
  public static final TunableNumber kickerSlowSpeed =
      new TunableNumber("Subsystems/Shooter/kickerSpeed", 0);
}
