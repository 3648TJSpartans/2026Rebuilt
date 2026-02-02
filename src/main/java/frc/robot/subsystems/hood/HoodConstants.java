package frc.robot.subsystems.hood;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class HoodConstants {
  public static final TunableNumber hoodEncoderFactor =
      new TunableNumber("Subsystems/Hood/rotationFactor", 1.0);
  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Hood/MotorIO")
          .motorCan(17)
          .p(0)
          .i(0)
          .d(0)
          .positionTolerance(0.01);
  public static final TunableNumber hoodTestSpeed =
      new TunableNumber("Subsystems/Hood/testSpeed", .05);

  public static final TunableNumber hoodAngleOffset =
      new TunableNumber("Subsystems/Hood/angleOffset", Math.PI / 2);

  public static final TunableNumber minAngle =
      new TunableNumber("Subsystems/Hood/limits/minAngle", 0.0);
  public static final TunableNumber maxAngle =
      new TunableNumber("Subsystems/Hood/limits/minAngle", 1.0);
}
