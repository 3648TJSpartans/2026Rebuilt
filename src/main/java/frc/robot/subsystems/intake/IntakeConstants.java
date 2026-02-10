package frc.robot.subsystems.intake;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class IntakeConstants {

  public static final MotorConfig hopperMotorConfig =
      new MotorConfig("Subsystems/Intake/HopperMotorIO")
          .motorCan(15)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final MotorConfig intakeRollerConfig =
      new MotorConfig("Subsystems/Intake/MotorIO")
          .motorCan(16)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final TunableNumber intakeRollerSpeed =
      new TunableNumber("Subsystems/Intake/intakeRollerSpeed", 0.1);
  public static final TunableNumber hopperSpeed =
      new TunableNumber("Subsystems/Intake/hopperSpeed", -.5);
  // Ideally the hopper is slowly spinning even when the robot isn't intaking or shooting
  // to push any balls in the hopper towards the shooter. It can't run fast or else
  // it will shred the balls.
  public static final TunableNumber hopperSlowSpeed =
      new TunableNumber("Subsystems/Intake/hopperSlowSpeed", 0);

  public static final int solenoidChannel = 1;
}
