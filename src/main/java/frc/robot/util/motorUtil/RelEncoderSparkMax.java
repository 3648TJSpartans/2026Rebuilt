package frc.robot.util.motorUtil;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Status;
import org.littletonrobotics.junction.Logger;

public class RelEncoderSparkMax extends MotorIO {

  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController motorController;
  private String name;
  private double m_positionTolerance;
  private double m_speedTolerance;
  private MotorConfig m_motorConfig;
  private double m_Ks;
  private double m_Kv;

  public RelEncoderSparkMax(MotorConfig motorConfig) {
    super(motorConfig.name());
    m_motorConfig = motorConfig;
    motor = new SparkMax(motorConfig.motorCan(), MotorType.kBrushless);
    motorController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    name = getName();
    configureMotor(motorConfig);
  }

  @Override
  public void setPosition(double setpoint) {
    super.setPosition(setpoint);
    motorController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setSpeed(double speed) {
    super.setSpeed(speed);
    motorController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

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

  @Override
  public void setEncoder(double setpoint) {
    encoder.setPosition(setpoint);
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
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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
    if (m_motorConfig.followCan() != 0) {
      config.follow(m_motorConfig.followCan(), m_motorConfig.isInverted());
    }
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_positionTolerance = m_motorConfig.positionTolerance();
    m_speedTolerance = m_motorConfig.speedTolerance();
    m_Ks = m_motorConfig.Ks();
    m_Kv = m_motorConfig.Kv();
  }

  public void configureMotor(MotorConfig motorConfig) {
    m_motorConfig = motorConfig;
    configureMotor();
  }

  public void runFFVelocity(double velocityRadPerSec) {
    super.setSpeed(velocityRadPerSec);
    double ffVolts = m_Ks * Math.signum(velocityRadPerSec) + m_Kv * velocityRadPerSec;
    motorController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return encoder.getVelocity();
  }

  // Runs specified voltage.
  public void runCharacterization(double output) {
    motor.setVoltage(output);
  }

  public Status getStatus() {
    if (motor.getLastError() == REVLibError.kOk) {
      return Status.OK;
    }
    Logger.recordOutput("Debug/" + name + "/revError", motor.getLastError().toString());
    return Status.ERROR;
  }
}
