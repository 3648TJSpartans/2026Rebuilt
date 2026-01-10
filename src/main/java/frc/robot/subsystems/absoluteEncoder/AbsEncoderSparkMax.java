package frc.robot.subsystems.absoluteEncoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.util.TunableNumber;

import com.revrobotics.spark.SparkClosedLoopController;

public class AbsEncoderSparkMax implements AbsEncoderIO {

    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController motorController;

    public AbsEncoderSparkMax() {
        motor = new SparkMax(AbsEncoderConstants.absEncoderMotorCan, MotorType.kBrushless);
        motorController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();

        var config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new TunableNumber("absEncoder/kP", AbsEncoderConstants.kP).get(),
                        new TunableNumber("absEncoder/kI", AbsEncoderConstants.kI).get(),
                        new TunableNumber("absEncoder/kD", AbsEncoderConstants.kD).get(),
                        new TunableNumber("absEncoder/kFF", AbsEncoderConstants.kFF).get())
                .outputRange(AbsEncoderConstants.kMinRange, AbsEncoderConstants.kMaxRange);
        config.signals.absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs(
                        (int) (1000.0 / AbsEncoderConstants.kAbsEncoderOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public void setTo(double setpoint) {
        motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void updateValues() {}

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
