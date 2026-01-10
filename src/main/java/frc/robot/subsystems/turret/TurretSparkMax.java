package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
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

public class TurretSparkMax implements TurretIO {

    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController motorController;

    public TurretSparkMax() {
        motor = new SparkMax(TurretConstants.absEncoderMotorCan, MotorType.kBrushless);
        motorController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();

        var config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new TunableNumber("turret/kP", TurretConstants.kP).get(),
                        new TunableNumber("turret/kI", TurretConstants.kI).get(),
                        new TunableNumber("turret/kD", TurretConstants.kD).get(),
                        new TunableNumber("turret/kFF", TurretConstants.kFF).get())
                .outputRange(TurretConstants.kMinRange, TurretConstants.kMaxRange)
                .positionWrappingEnabled(TurretConstants.positionWrapping);
        config.signals.absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs(
                        (int) (1000.0 / TurretConstants.kAbsEncoderOdometryFrequency))
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
        Logger.recordOutput("Turret/Running", true);
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
