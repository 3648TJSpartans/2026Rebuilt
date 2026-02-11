package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class TurretConstants {

  public static final MotorConfig kTurretMotorConfig =
      new MotorConfig("Subsystems/Turret/MotorIO/")
          .motorCan(12)
          .p(2.0)
          .d(0.0)
          .i(0.0)
          .positionTolerance(0.3)
          .maxPower(0.3)
          .minPower(-0.3);
  public static final Translation3d kTurretOffset = new Translation3d(-.26, .26, .58);
  public static final TunableNumber kTurretMinRotation =
      new TunableNumber("Subsystems/Turret/minRotation", -Math.PI);
  public static final TunableNumber kTurretMaxRotation =
      new TunableNumber("Subsystems/Turret/maxRotation", Math.PI);
  public static double encoderPositionFactor = Math.PI / 63;
  public static final int zeroSwitchPort = 0;
}
