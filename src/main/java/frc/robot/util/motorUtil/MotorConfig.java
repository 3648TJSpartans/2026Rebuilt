package frc.robot.util.motorUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.TunableNumber;

public class MotorConfig {
  private static final double DEFAULT_POSITION_TOLERANCE = 0.0;
  private static final double DEFAULT_SPEED_TOLERANCE = 0.0;
  private static final double DEFAULT_P = 0.0;
  private static final double DEFAULT_I = 0.0;
  private static final double DEFAULT_D = 0.0;
  private static final double DEFAULT_FF = 0.0;
  private static final double DEFAULT_MIN_POWER = -1.0;
  private static final double DEFAULT_MAX_POWER = 1.0;
  private static final double DEFAULT_ODOMETRY_FREQUENCY = 100;
  private static final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;
  private static final double DEFAULT_Ks = 0.0;
  private static final double DEFAULT_Kv = 0.0;

  private String m_loggingName = "Subsystems/MotorIOs/defaultMotor";
  private int m_motorCan = 1;

  private TunableNumber m_positionTolerance;
  private TunableNumber m_speedTolerance;
  private TunableNumber m_P;
  private TunableNumber m_I;
  private TunableNumber m_D;
  private TunableNumber m_FF;
  private TunableNumber m_minPower;
  private TunableNumber m_maxPower;
  private TunableNumber m_Ks;
  private TunableNumber m_Kv;
  private IdleMode m_IdleMode = DEFAULT_IDLE_MODE;
  private double m_encoderOdometryFrequency = DEFAULT_ODOMETRY_FREQUENCY;
  private boolean m_isInverted = false;

  public MotorConfig(String name) {
    m_loggingName = name;
  }

  public MotorConfig name(String name) {
    m_loggingName = name;
    return this;
  }

  public MotorConfig motorCan(int motorCAN) {
    m_motorCan = (int) new TunableNumber(m_loggingName + "/motorCAN", motorCAN).get();
    return this;
  }

  public MotorConfig positionTolerance(double positionTolerance) {
    m_positionTolerance =
        new TunableNumber(m_loggingName + "/Tolerances/positionTolerance", positionTolerance);
    return this;
  }

  public MotorConfig speedTolerance(double speedTolerance) {
    m_speedTolerance =
        new TunableNumber(m_loggingName + "/Tolerances/speedTolerance", speedTolerance);
    return this;
  }

  public MotorConfig p(double p) {
    m_P = new TunableNumber(m_loggingName + "/PIDF/P", p);
    return this;
  }

  public MotorConfig i(double i) {
    m_I = new TunableNumber(m_loggingName + "/PIDF/I", i);
    return this;
  }

  public MotorConfig d(double d) {
    m_D = new TunableNumber(m_loggingName + "/PIDF/D", d);
    return this;
  }

  public MotorConfig ff(double ff) {
    m_FF = new TunableNumber(m_loggingName + "/PIDF/FF", ff);
    return this;
  }

  public MotorConfig minPower(double minPower) {
    m_minPower = new TunableNumber(m_loggingName + "/PowerRange/minPower", minPower);
    return this;
  }

  public MotorConfig maxPower(double maxPower) {
    m_maxPower = new TunableNumber(m_loggingName + "/PowerRange/maxPower", maxPower);
    return this;
  }

  public MotorConfig encoderOdometryFrequency(double encoderOdometryFrequency) {
    m_encoderOdometryFrequency =
        new TunableNumber(m_loggingName + "/EncoderOdometryFrequency", encoderOdometryFrequency)
            .get();
    return this;
  }

  public MotorConfig idleMode(IdleMode idleMode) {
    m_IdleMode = idleMode;
    return this;
  }

  public MotorConfig Ks(double Ks) {
    m_Ks = new TunableNumber(m_loggingName + "/FF/Ks", Ks);
    return this;
  }

  public MotorConfig Kv(double Kv) {
    m_Kv = new TunableNumber(m_loggingName + "/FF/Kv", Kv);
    return this;
  }

  // Not Tunable
  // TODO change with Tunable Boolean if you want to.
  public MotorConfig isInverted(boolean isInverted) {
    m_isInverted = isInverted;
    return this;
  }

  public String name() {
    return m_loggingName;
  }

  public int motorCan() {
    return m_motorCan;
  }

  public double positionTolerance() {
    if (m_positionTolerance == null) {
      return DEFAULT_POSITION_TOLERANCE;
    }
    return m_positionTolerance.get();
  }

  public double speedTolerance() {
    if (m_speedTolerance == null) {
      return DEFAULT_SPEED_TOLERANCE;
    }
    return m_speedTolerance.get();
  }

  public double p() {
    if (m_P == null) {
      return DEFAULT_P;
    }
    return m_P.get();
  }

  public double i() {
    if (m_I == null) {
      return DEFAULT_I;
    }
    return m_I.get();
  }

  public double d() {
    if (m_D == null) {
      return DEFAULT_D;
    }
    return m_D.get();
  }

  public double ff() {
    if (m_FF == null) {
      return DEFAULT_FF;
    }
    return m_FF.get();
  }

  public double minPower() {
    if (m_minPower == null) {
      return DEFAULT_MIN_POWER;
    }
    return m_minPower.get();
  }

  public double maxPower() {
    if (m_maxPower == null) {
      return DEFAULT_MAX_POWER;
    }
    return m_maxPower.get();
  }

  public IdleMode idleMode() {
    return m_IdleMode;
  }

  public double Ks() {
    if (m_Ks == null) {
      return DEFAULT_Ks;
    }
    return m_Ks.get();
  }

  public double Kv() {
    if (m_Kv == null) {
      return DEFAULT_Kv;
    }
    return m_Kv.get();
  }

  public double encoderOdometryFrequency() {
    return m_encoderOdometryFrequency;
  }

  public boolean isInverted() {
    return m_isInverted;
  }
}
