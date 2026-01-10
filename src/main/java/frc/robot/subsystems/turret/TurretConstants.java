package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
    public static final int absEncoderMotorCan = 13;

    public static final double kP = .75;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = .005;

    public static boolean positionWrapping = true;

    public static final double kMinRange = -0.2;
    public static final double kMaxRange = 0.2;

    public static final double setpoint1 = 0;
    public static final double setpoint2 = 1;

    public static final double setpointTolerance = 0.05;

    public static final double kAbsEncoderOdometryFrequency = 100;
    public static final boolean encoderInverted = false;
    public static final double encoderPositionFactor = 1;

    public static final Rotation2d rotationOffset = new Rotation2d(Math.PI);
}
