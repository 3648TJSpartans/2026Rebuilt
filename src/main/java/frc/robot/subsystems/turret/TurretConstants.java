package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
          .maxPower(0.5)
          .minPower(-0.5);
  public static final Translation3d kTurretOffset = new Translation3d(-.165, .165, .39);
  public static final TunableNumber kTurretMinPose =
      new TunableNumber("Subsystems/Turret/minPosition", -50);
  public static final TunableNumber kTurretMaxPose =
      new TunableNumber("Subsystems/Turret/maxPosition", 50);
  public static double encoderPositionFactor = Math.PI / 61.02;
  public static TunableNumber kVSim = new TunableNumber("Subsystems/Turret/Sim/kV", 100.0);
  public static final int zeroSwitchPort = 0;
  public static final TunableNumber turretZeroingOffset =
      new TunableNumber("Subsystems/Turret/turretZeroingOffset", -9.66);

  public static final double homePower =
      new TunableNumber("Subsystems/Turret/homePower", 0.1).get();
  public static final double homeRange =
      Units.degreesToRadians(new TunableNumber("Subsystems/Turret/homeRange (deg)", 30).get());

  public static final TunableNumber rotationOffset =
      new TunableNumber("Subsystems/Turret/rotationOffset (rad)", Math.PI/2);
}
