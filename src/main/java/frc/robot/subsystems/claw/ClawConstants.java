package frc.robot.subsystems.claw;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ClawConstants {

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Claw/MotorIO")
          .motorCan(19)
          .p(0.0)
          .d(0.0)
          .i(0.0)
          .maxPower(0.1)
          .minPower(-0.1)
          .positionTolerance(0.0);
  public static final int bottomSwitchPort = 5;
  public static final int topSwitchPort = 6;
  public static final TunableNumber minPosition =
      new TunableNumber("Subsystems/Claw/MinPosition", 0.0);
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Claw/MaxPosition", 10.0);
  public static final double encoderPositionFactor = 1.0;
}
