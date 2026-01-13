package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Rotation2d;

public class Trajectory {
  private final double m_shooterAngle;
  private final double m_turretAngle;
  private final double m_shooterSpeed;
  private final double m_hangTime;
  

  public Trajectory( double shooterAngle,double turretAngle, double shooterSpeed,double hangTime){
    m_shooterAngle = shooterAngle;
    m_turretAngle = turretAngle;
    m_shooterSpeed = shooterSpeed;
    m_hangTime = hangTime;
  }

  public double getShooterAngle(){
    return m_shooterAngle;
  }
  public double getTurretAngle(){
    return m_turretAngle;
  } 
  public double getShooterSpeed(){
    return m_shooterSpeed;
  }
  public double getHangTime(){
    return m_hangTime;
  }

  public Rotation2d getTurretRotation(){
    return new Rotation2d(m_turretAngle);
  }

  public String toString(){
    return "Shooter Angle: "+m_shooterAngle+"\nTurret Angle: "+m_turretAngle+"\nShooter Speed: "+m_shooterSpeed+"\nHang Time: "+m_hangTime;
  }
}
