package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class TurretConstants {

  public static final MotorConfig kTurretMotorConfig =
      new MotorConfig("Subsystems/Turret/MotorIO/")
          .motorCan(13)
          .p(1.0)
          .d(0.0)
          .i(0.0)
          .maxPower(0.05)
          .minPower(-0.05);
  public static final Translation3d kTurretOffset = new Translation3d(-.3, .2, .2);
  public static final TunableNumber kTurretMinRotation =
      new TunableNumber("Subsystems/Turret/minRotation", -3 * Math.PI / 4);
  public static final TunableNumber kTurretMaxRotation =
      new TunableNumber("Subsystems/Turret/maxRotation", 3 * Math.PI / 4);
  public static double encoderPositionFactor = 2 * Math.PI / 5.23;

  public static final int zeroSwitchPort = 0;

  public static final double homePower =
      new TunableNumber("Subsystems/Turret/homePower", 0.1).get();
  public static final double homeRange =
      Units.degreesToRadians(new TunableNumber("Subsystems/Turret/homeRange (deg)", 30).get());
}
