package frc.robot.subsystems.relativeEncoder;

import frc.robot.util.TunableNumber;

/*
 * Constants files contain numbers and other values
 * that shouldn't change while robot code is running,
 * except for tuning.
 */

public class RelEncoderConstants {

    public static final int relEncoderMotorCan = 14;

    public static final double setpoint1 = new TunableNumber("relEncoder/setpoint1", 0).get();
    public static final double setpoint2 = new TunableNumber("relEncoder/setpoint2", 1).get();
    public static final double relativeEncoderLimit = 2;

    public static final double kP = new TunableNumber("relEncoder/kP", 0).get();
    public static final double kI = new TunableNumber("relEncoder/kI", 0).get();
    public static final double kD = new TunableNumber("relEncoder/kD", 0).get();
    public static final double kFF = new TunableNumber("relEncoder/kFF", 0).get();
    public static final double odometryFrequency = 100;
    public static final double kMinRange = -1;
    public static final double kMaxRange = 1;
    public static final boolean encoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;

    public static final int limitSwitchPin = 1;

    public static final double marginOfError = new TunableNumber("relEncoder/marginOfError", 0).get();

}
