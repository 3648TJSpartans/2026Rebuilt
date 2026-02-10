package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class HoodConstants {

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Hood/MotorIO")
          .motorCan(17)
          .p(0)
          .i(0)
          .d(0)
          .positionTolerance(0.01)
          .isInverted(true);
  public static final TunableNumber hoodTestSpeed =
      new TunableNumber("Subsystems/Hood/testSpeed", .05);

  public static final TunableNumber hoodAngleOffset =
      new TunableNumber("Subsystems/Hood/angleOffset", Math.PI / 2);
  public static final TunableNumber minPosition =
      new TunableNumber("Subsystems/Hood/limits/minPose", 0.02);
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Hood/limits/maxPose", 0.57);
  public static final Rotation2d minAngle = new Rotation2d(Units.degreesToRadians(69));
  public static final Rotation2d maxAngle = new Rotation2d(Units.degreesToRadians(79.8));
  public static final double hoodEncoderFactor =
      maxPosition.get() / (maxAngle.getRadians() - minAngle.getRadians());
}
