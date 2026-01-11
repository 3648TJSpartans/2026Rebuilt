package frc.robot.subsystems.simpleMotor;

import frc.robot.util.TunableNumber;

/*
 * Constants files contain numbers and other values
 * that shouldn't change while robot code is running,
 * except for tuning.
 */

public class SimpleMotorConstants {

  /*
   * Since motors are controlled by CANs, they each have IDs for which CAN to use.
   */

  public static final int simpleMotorCan = 11;

  /*
   * TunableNumbers allow constants that need to be adjusted to be tuned while the
   * robot is running.
   */

  public static final double speed1 = new TunableNumber("simpleMotor/speed1", 0).get();
  public static final double speed2 = new TunableNumber("simpleMotor/speed2", 0.1).get();
}
