package frc.robot.subsystems.climber;

import frc.robot.util.motorUtil.MotorConfig;

public class ClimberConstants {

  public static final int climbMotorCan = 23;

  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;
  public static final double kOdometryFrequency = 100;
  public static final double kMinRange = -0.1;
  public static final double kMaxRange = 0.1;
  public static final boolean kEncoderInverted = false;
  public static final double elevatorEncoderPositionFactor = 1;
  public static final double elevatorEncoderVelocityFactor = 1;

  public static final double kPositionTolerance = 0.05;
  public static final double kSpeedTolerance = 0.05;

  public static final MotorConfig climbMotorConfig =
      new MotorConfig("exampleSubsystem/motor1")
          .motorCan(climbMotorCan)
          .p(kP)
          .i(kI)
          .d(kD)
          .ff(kFF)
          .encoderOdometryFrequency(kOdometryFrequency)
          .minPower(kMinRange)
          .maxPower(kMaxRange)
          .isInverted(kEncoderInverted)
          .positionTolerance(kPositionTolerance)
          .speedTolerance(kSpeedTolerance);
}
