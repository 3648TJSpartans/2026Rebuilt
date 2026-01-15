package frc.robot.subsystems.climber;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ClimberConstants {
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Climber/Limits/maxPosition", 1.0);
  public static final TunableNumber minPosition =
      new TunableNumber("Subsystems/Climber/Limits/minPosiiton", 0.0);

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Climber/MotorIO")
          .motorCan(16)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final double encoderPositionFactor = 1.0;

  public static final int bottomSwitchPort = 2;
  public static final int topSwitchPort = 3;
}
