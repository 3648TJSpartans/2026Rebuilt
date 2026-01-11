package frc.robot.util.motorUtil;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AbsEncoderSparkMax extends MotorIO {

  private MotorConfig m_motorConfig;
  private SparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkClosedLoopController motorController;
  private String name;
  private double m_positionTolerance;
  private double m_speedTolerance;

  public AbsEncoderSparkMax(MotorConfig motorConfig) {
    super(motorConfig.name());

    m_motorConfig = motorConfig;
    motor = new SparkMax(motorConfig.motorCan(), MotorType.kBrushless);
    motorController = motor.getClosedLoopController();
    encoder = motor.getAbsoluteEncoder();
    name = getName();
    configureMotor(motorConfig);
  }

  @Override
  public void setPosition(double setpoint) {
    super.setPosition(setpoint);
    motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpeed(double speed) {
    super.setSpeed(speed);
    motorController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPower(double power) {
    super.setPower(power);
    motor.set(power);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getSpeed() {
    return encoder.getVelocity();
  }

  // Absolute Encoders cannot reset
  @Override
  public void setEncoder(double setpoint) {
    return;
  }

  public double getPositionTolerance() {
    return m_positionTolerance;
  }

  public double getSpeedTolerance() {
    return m_speedTolerance;
  }

  @Override
  public void configureMotor() {
    var config = new SparkMaxConfig();
    config
        .inverted(m_motorConfig.isInverted())
        .idleMode(m_motorConfig.idleMode())
        .voltageCompensation(12.0);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(m_motorConfig.p(), m_motorConfig.i(), m_motorConfig.d(), m_motorConfig.ff())
        .outputRange(m_motorConfig.minPower(), m_motorConfig.maxPower());
    config
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / m_motorConfig.encoderOdometryFrequency()))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_positionTolerance = m_motorConfig.positionTolerance();
    m_speedTolerance = m_motorConfig.speedTolerance();
  }

  public void configureMotor(MotorConfig motorConfig) {
    m_motorConfig = motorConfig;
    configureMotor();
  }
}
