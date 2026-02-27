package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Rotation2d;

public class Trajectory {
  private final double m_shooterAngle;
  private final double m_turretAngle;
  private final double m_shooterSpeed;
  private final double m_hangTime;
  private final boolean m_isValid;
  private double m_error;

  private Trajectory(
      double shooterAngle,
      double turretAngle,
      double shooterSpeed,
      double hangTime,
      boolean isValid,
      double error) {
    m_shooterAngle = shooterAngle;
    m_turretAngle = turretAngle;
    m_shooterSpeed = shooterSpeed;
    m_hangTime = hangTime;
    m_isValid = isValid;
    m_error = error;
  }

  public Trajectory(double shooterAngle, double turretAngle, double shooterSpeed, double hangTime,double error) {
    this(shooterAngle, turretAngle, shooterSpeed, hangTime, true, error);
  }
  public Trajectory(double shooterAngle, double turretAngle, double shooterSpeed, double hangTime) {
    this(shooterAngle, turretAngle, shooterSpeed, hangTime, true, 0.0);
  }

  public static Trajectory invalidTrajectory = new Trajectory(0, 0, 0, 0, false, Double.MAX_VALUE);

  public double getShooterAngle() {
    return m_shooterAngle;
  }

  public double getTurretAngle() {
    return m_turretAngle;
  }

  public double getShooterSpeed() {
    return m_shooterSpeed;
  }

  public double getHangTime() {
    return m_hangTime;
  }

    public double getError(){
      return m_error;
    }

    public void setError(double error){
      m_error = error;
    }
  public Rotation2d getTurretRotation() {
    return new Rotation2d(m_turretAngle);
  }

  public boolean isValid() {
    return m_isValid;
  }

  public Trajectory plus(Trajectory other) {
    return new Trajectory(
        m_shooterAngle + other.m_shooterAngle,
        m_turretAngle + other.m_turretAngle,
        m_shooterSpeed + other.m_shooterSpeed,
        m_hangTime + other.m_hangTime,
        m_error + other.m_error);
  }

  public String toString() {
    return "Shooter Angle: "
        + m_shooterAngle
        + "\nTurret Angle: "
        + m_turretAngle
        + "\nShooter Speed: "
        + m_shooterSpeed
        + "\nHang Time: "
        + m_hangTime;
  }

  public Trajectory copy() {
    return new Trajectory(m_shooterAngle, m_turretAngle, m_shooterSpeed, m_hangTime);
  }
}
