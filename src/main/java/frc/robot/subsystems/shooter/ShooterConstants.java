package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.MotorConfig;

public class ShooterConstants {

  public static final int turretMotorCan = 20;
  public static final int pivotMotorCan = 21;
  public static final int shootMotorCan = 22;

  public static final double turretkP = 0.5;
  public static final double turretkI = 0.0;
  public static final double turretkD = 0.0;
  public static final double turretkFF = 0.0;
  public static final double turretkOdometryFrequency = 100;
  public static final double turretkMinRange = -0.1;
  public static final double turretkMaxRange = 0.1;
  public static final boolean turretkEncoderInverted = false;
  public static final double turretElevatorEncoderPositionFactor = 1;
  public static final double turretElevatorEncoderVelocityFactor = 1;

  public static final double turretkPositionTolerance = 0.05;
  public static final double turretkSpeedTolerance = 0.05;

  public static final double pivotkP = 0.5;
  public static final double pivotkI = 0.0;
  public static final double pivotkD = 0.0;
  public static final double pivotkFF = 0.0;
  public static final double pivotkOdometryFrequency = 100;
  public static final double pivotkMinRange = -0.1;
  public static final double pivotkMaxRange = 0.1;
  public static final boolean pivotkEncoderInverted = false;
  public static final double pivotElevatorEncoderPositionFactor = 1;
  public static final double pivotElevatorEncoderVelocityFactor = 1;

  public static final double pivotkPositionTolerance = 0.05;
  public static final double pivotkSpeedTolerance = 0.05;

  public static final MotorConfig turretMotorConfig =
      new MotorConfig("exampleSubsystem/motor1")
          .motorCan(turretMotorCan)
          .p(turretkP)
          .i(turretkI)
          .d(turretkD)
          .ff(turretkFF)
          .encoderOdometryFrequency(turretkOdometryFrequency)
          .minPower(turretkMinRange)
          .maxPower(turretkMaxRange)
          .isInverted(turretkEncoderInverted)
          .positionTolerance(turretkPositionTolerance)
          .speedTolerance(turretkSpeedTolerance);

  public static final MotorConfig pivotMotorConfig =
      new MotorConfig("exampleSubsystem/motor1")
          .motorCan(pivotMotorCan)
          .p(pivotkP)
          .i(pivotkI)
          .d(pivotkD)
          .ff(pivotkFF)
          .encoderOdometryFrequency(pivotkOdometryFrequency)
          .minPower(pivotkMinRange)
          .maxPower(pivotkMaxRange)
          .isInverted(pivotkEncoderInverted)
          .positionTolerance(pivotkPositionTolerance)
          .speedTolerance(pivotkSpeedTolerance);
}
