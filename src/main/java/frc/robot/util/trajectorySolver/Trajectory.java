package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Rotation2d;

public class Trajectory {
  private final double m_shooterAngle;
  private final double m_turretAngle;
  private final double m_shooterSpeed;
  private final double m_hangTime;
  private final boolean m_isValid;

  private Trajectory(
      double shooterAngle,
      double turretAngle,
      double shooterSpeed,
      double hangTime,
      boolean isValid) {
    m_shooterAngle = shooterAngle;
    m_turretAngle = turretAngle;
    m_shooterSpeed = shooterSpeed;
    m_hangTime = hangTime;
    m_isValid = isValid;
  }

  public Trajectory(double shooterAngle, double turretAngle, double shooterSpeed, double hangTime) {
    this(shooterAngle, turretAngle, shooterSpeed, hangTime, true);
  }

  public static Trajectory invalidTrajectory = new Trajectory(0, 0, 0, 0, false);

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

  public Rotation2d getTurretRotation() {
    return new Rotation2d(m_turretAngle);
  }

  public boolean isValid() {
    return m_isValid;
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
}
