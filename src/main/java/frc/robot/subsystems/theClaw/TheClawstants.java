package frc.robot.subsystems.theClaw;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class TheClawstants {

  public static final int bottomSwitchPort = 4;
  public static final double encoderPositionFactor = 0.1;
  public static final TunableNumber minPosition = new TunableNumber("Subsystems/Claw/minPose", 0.1);
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Claw/maxPose", 10.0);
  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Claw/MotorIO")
          .motorCan(21)
          .p(0.0)
          .i(0.0)
          .d(0.0)
          .positionTolerance(0.3)
          .maxPower(0.3)
          .minPower(-0.3);

  public static final TunableNumber simKv = new TunableNumber("Subsystems/Claw/simKv", 1.0);
}
