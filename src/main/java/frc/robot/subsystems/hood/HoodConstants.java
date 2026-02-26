package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;
import java.util.function.DoubleSupplier;

public class HoodConstants {

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Hood/MotorIO")
          .motorCan(14)
          .p(2)
          .i(0.01)
          .d(0)
          .minPower(-.2)
          .maxPower(0.2)
          .positionTolerance(0.01)
          .isInverted(true);
  public static final TunableNumber hoodTestSpeed =
      new TunableNumber("Subsystems/Hood/testSpeed", .05);

  public static final TunableNumber hoodAngleOffset =
      new TunableNumber("Subsystems/Hood/angleOffset", Math.PI / 2);
  public static final TunableNumber minPosition =
      new TunableNumber("Subsystems/Hood/limits/minPose", 0.04);
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Hood/limits/maxPose", 0.57);
  public static final TunableNumber minAngle =
      new TunableNumber("Subsystems/Hood/minAngle (deg)", 78);
  //   new Rotation2d(
  //       Units.degreesToRadians(
  //           78)); // This ends up being the max angle we shoot, the min angle the hood is at.
  public static final TunableNumber maxAngle =
      new TunableNumber("Subsystems/Hood/maxAngle (deg)", 60);
  public static final TunableNumber simKV = new TunableNumber("Subsystems/Hood/maxAngle (deg)", 1);
  //   new Rotation2d(Units.degreesToRadians(60));
  public static final DoubleSupplier hoodEncoderFactor =
      () -> maxPosition.get() / (Units.degreesToRadians(maxAngle.get() - minAngle.get()));
  public static final TunableNumber headUpAngle =
      new TunableNumber("Subsystems/Hood/headUpAngle", 0.2);
}
