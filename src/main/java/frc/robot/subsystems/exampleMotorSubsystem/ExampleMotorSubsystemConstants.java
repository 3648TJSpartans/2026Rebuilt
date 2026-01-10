package frc.robot.subsystems.exampleMotorSubsystem;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.*;
/*
 * Constants files contain numbers and other values that shouldn't change while robot code is
 * running, except for tuning.
 */

public class ExampleMotorSubsystemConstants {

        public static final int motor1Can = 18;
        public static final int motor2Can = 17;

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

        public static final int limitSwitchPin = 1;

        public static final double kPositionTolerance = 0.05;
        public static final double kSpeedTolerance = 0.05;

        public static final double power = 0.1;

        public static final MotorConfig motor1Config = new MotorConfig("exampleSubsystem/motor1")
                        .motorCan(motor1Can).p(kP).i(kI).d(kD).ff(kFF)
                        .encoderOdometryFrequency(kOdometryFrequency).minPower(kMinRange)
                        .maxPower(kMaxRange).isInverted(kEncoderInverted)
                        .positionTolerance(kPositionTolerance).speedTolerance(kSpeedTolerance);

        // Copies motor 1 config except for changing name and CAN ID.
        // For motors that need different PID values, just make a second
        // set of constants and a second config

        public static final MotorConfig motor2Config = new MotorConfig("exampleSubsystem/motor2")
                        .motorCan(motor2Can).p(kP).i(kI).d(kD).ff(kFF)
                        .encoderOdometryFrequency(kOdometryFrequency).minPower(kMinRange)
                        .maxPower(kMaxRange).isInverted(kEncoderInverted)
                        .positionTolerance(kPositionTolerance).speedTolerance(kSpeedTolerance);



        public static final MotorIO motor1 = new AbsEncoderSparkMax(motor1Config);
        public static final MotorIO motor2 = new RelEncoderSparkMax(motor2Config);
}
