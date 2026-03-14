package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ShooterConstants {
  public static MotorConfig kLeaderMotorConfig =
      new MotorConfig("Subsystems/Shooter/LeaderMotorIO")
          .motorCan(11)
          .Ks(-0.0516)
          .Kv(0.00182)
          .minPower(0.0)
          .speedTolerance(100)
          .idleMode(IdleMode.kCoast);
  public static MotorConfig kFollowerMotorConfig =
      new MotorConfig("Subsystems/Shooter/FollowerMotorIO")
          .motorCan(10)
          .Ks(-0.0516)
          .Kv(0.00182)
          .maxPower(0.0)
          .speedTolerance(100)
          .idleMode(IdleMode.kCoast);
  //   .follow(10)
  //   .isInverted(true);
  public static final TunableNumber shooterSimKV =
      new TunableNumber("Subsystems/Shooter/kV", 6000.0);
  public static MotorConfig kKickerMotorConfig =
      new MotorConfig("Subsystems/Kicker/MotorIO").motorCan(13).Ks(0).Kv(0).speedTolerance(0.0);

  public static final int kickerIRSensorChannel = 3;

  /*Used to convert from RPM to m/s the ball is shot. RPM*s/m
   * Example: If revving the shooter to 5000 rpm results in a ball velocity of 15 m/s, then the factor will be 5000/15 = 333.33 RPM*s/m
   */
  public static final TunableNumber kShooterVelocityFactor =
      new TunableNumber("Subsystems/Shooter/VelocityFactor", 580);
  public static final TunableNumber rpmThreshold =
      new TunableNumber("Subsystems/Shooter/RPMThreshold", -200);
  public static final TunableNumber kickerSpeed =
      new TunableNumber("Subsystems/Kicker/kickerSpeed", 1.0);
  public static final TunableNumber kickerSlowSpeed =
      new TunableNumber("Subsystems/Kicker/kickerSpeed", 0);
  public static final TunableNumber shootHeadUpSpeed =
      new TunableNumber("Subsystems/Shooter/shootHeadUpSpeed", 300);
}
