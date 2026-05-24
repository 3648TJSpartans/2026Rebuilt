package frc.robot.util.motorUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumberAutoUpdater;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class MotorConfig {
  // private static int testCount = 0;
  private static final Set<String> motorNames = new HashSet<>();

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
  private static final int DEFAULT_FOLLOW_CAN = 0;

  private String m_loggingName = "Subsystems/MotorIOs/defaultMotor";
  private int m_motorCan = 1;

  private Optional<ConfigurableMotor> m_linkedMotor;

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
  private TunableNumber m_followCan;

  public MotorConfig(String name) {
    m_loggingName = name;
    if (motorNames.contains(name)) {
      throw new IllegalArgumentException(
          "MotorConfig " + name + " already exists. Please choose a different name.");
    }
    motorNames.add(name);
    m_linkedMotor = Optional.empty();
  }

  protected void linkMotor(ConfigurableMotor motor) {
    m_linkedMotor = Optional.of(motor);
  }

  public MotorConfig name(String name) {
    m_loggingName = name;
    return this;
  }

  public MotorConfig motorCan(int motorCAN) {
    m_motorCan =
        (int)
            new TunableNumberAutoUpdater(
                    m_loggingName + "/motorCAN", motorCAN, this::configureLinkedMotor)
                .get();
    return this;
  }

  public MotorConfig positionTolerance(double positionTolerance) {
    m_positionTolerance =
        new TunableNumberAutoUpdater(
            m_loggingName + "/Tolerances/positionTolerance",
            positionTolerance,
            this::configureLinkedMotor);
    return this;
  }

  public MotorConfig speedTolerance(double speedTolerance) {
    m_speedTolerance =
        new TunableNumberAutoUpdater(
            m_loggingName + "/Tolerances/speedTolerance",
            speedTolerance,
            this::configureLinkedMotor);
    return this;
  }

  public MotorConfig p(double p) {
    m_P = new TunableNumberAutoUpdater(m_loggingName + "/PIDF/P", p, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig i(double i) {
    m_I = new TunableNumberAutoUpdater(m_loggingName + "/PIDF/I", i, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig d(double d) {
    m_D = new TunableNumberAutoUpdater(m_loggingName + "/PIDF/D", d, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig ff(double ff) {
    m_FF = new TunableNumberAutoUpdater(m_loggingName + "/PIDF/FF", ff, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig minPower(double minPower) {
    m_minPower =
        new TunableNumberAutoUpdater(
            m_loggingName + "/PowerRange/minPower", minPower, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig maxPower(double maxPower) {
    m_maxPower =
        new TunableNumberAutoUpdater(
            m_loggingName + "/PowerRange/maxPower", maxPower, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig encoderOdometryFrequency(double encoderOdometryFrequency) {
    m_encoderOdometryFrequency =
        new TunableNumberAutoUpdater(
                m_loggingName + "/EncoderOdometryFrequency",
                encoderOdometryFrequency,
                this::configureLinkedMotor)
            .get();
    return this;
  }

  public MotorConfig idleMode(IdleMode idleMode) {
    m_IdleMode = idleMode;
    return this;
  }

  public MotorConfig Ks(double Ks) {
    m_Ks = new TunableNumberAutoUpdater(m_loggingName + "/FF/Ks", Ks, this::configureLinkedMotor);
    return this;
  }

  public MotorConfig Kv(double Kv) {
    m_Kv = new TunableNumberAutoUpdater(m_loggingName + "/FF/Kv", Kv, this::configureLinkedMotor);
    return this;
  }

  // Not Tunable
  // TODO change with Tunable Boolean if you want to.
  public MotorConfig isInverted(boolean isInverted) {
    m_isInverted = isInverted;
    return this;
  }

  public MotorConfig follow(int followCan) {
    m_followCan =
        new TunableNumberAutoUpdater(
            m_loggingName + "/followCan", followCan, this::configureLinkedMotor);
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

  public int followCan() {
    if (m_followCan == null) {
      return DEFAULT_FOLLOW_CAN;
    } else {
      return (int) m_followCan.get();
    }
  }

  private void configureLinkedMotor() {
    if (m_linkedMotor.isPresent()) {
      // long startTime = System.nanoTime();
      m_linkedMotor.get().configureMotor(this);
      // Logger.recordOutput("MotorConfig/configureTime", (System.nanoTime() - startTime) * 1e-9);
      // testCount++;
      // Logger.recordOutput("MotorConfig/testCount", testCount);
    }
    //  else {
    //   System.out.println("No motor linked to " + m_loggingName + " to configure.");
    // }
  }
}
