package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class ClimberConstants {
  public static final TunableNumber maxPosition =
      new TunableNumber("Subsystems/Climber/Limits/maxPosition", 1.0);
  public static final TunableNumber minPosition =
      new TunableNumber("Subsystems/Climber/Limits/minPosiiton", 0.0);

  public static final MotorConfig leadMotorConfig =
      new MotorConfig("Subsystems/Climber/MotorIOs/Leads")
          .motorCan(3)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);
  public static final MotorConfig followMotorConfig =
      new MotorConfig("Subsystems/Climber/MotorIOs/Follow")
          .motorCan(2)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);
  public static final double encoderPositionFactor = 1.0;

public static final TunableNumber powerToSpeedThreshold = new TunableNumber("Subsystems/Climber/powerToSpeedThreshold",0.015);

  public static final int bottomSwitchPort = 6;
  public static final int topSwitchPort = 5;

  public static final double autoClimbP = 0.62;
  public static final double autoClimbI = 0.0;
  public static final double autoClimbD = 0.0;
  public static final double autoMaxPower = .5;
  private static final double autoMinPower = 0.0;
  private static final double autoTolerance = 0.1;
  private static final double autoSetAngle = Math.PI / 2;
  public static final TunableNumber tunableAutoClimbP =
      new TunableNumber("Subsystems/Climber/Auto/PID/P", autoClimbP);
  public static final TunableNumber tunableAutoClimbI =
      new TunableNumber("Subsystems/Climber/Auto/PID/I", autoClimbI);
  public static final TunableNumber tunableAutoClimbD =
      new TunableNumber("Subsystems/Climber/Auto/PID/D", autoClimbD);
  public static final TunableNumber tunableAutoMaxPower =
      new TunableNumber("Subsystems/Climber/Auto/PID/MaxPower", autoMaxPower);
  public static final TunableNumber tunableAutoMinPower =
      new TunableNumber("Subsystems/Climber/Auto/PID/MinPower", autoMinPower);
  public static final TunableNumber tunablAutoTolerance =
      new TunableNumber("Subsystems/Climber/Auto/PID/tolerance (rad)", autoTolerance);
  public static final TunableNumber tunableGoal =
      new TunableNumber("Subsystems/Climber/Auto/PID/setAngle (rad)", autoSetAngle);

  public static PIDController getPidController() {
    PIDController out =
        new PIDController(
            tunableAutoClimbP.get(), tunableAutoClimbI.get(), tunableAutoClimbD.get());
    // Stops violent jumps
    out.enableContinuousInput(-Math.PI, Math.PI);
    return out;
  }
}
