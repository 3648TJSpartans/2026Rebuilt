package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Turret extends RelEncoderSparkMax{
  private final Translation3d m_turretOffset;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;
  private Pose3d turretPose;
  private double[] turretTranslationalVelocity;
  public Turret(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
    super(TurretConstants.kTurretMotorConfig);
    m_turretOffset = TurretConstants.kTurretOffset;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    turretPose = new Pose3d();
    turretTranslationalVelocity = new double[2];
  }

  @AutoLogOutput(key = "Subsystems/Turret/TurretAngle")
  public Rotation2d getTurretRotation(){
    return new Rotation2d(getPosition() * TurretConstants.encoderPositionFactor);
  }
  
  @Override
  public void periodic(){
    super.periodic();
    updateInputs();
  }

  public void updateInputs(){
    Pose2d robotPose = m_robotPoseSupplier.get();
    double xr = robotPose.getX();
    double yr = robotPose.getY();
    double thetar = robotPose.getRotation().getRadians();
    double xt = m_turretOffset.getX() * Math.cos(thetar) - m_turretOffset.getY() * Math.sin(thetar) + xr;
    double yt = m_turretOffset.getX() * Math.sin(thetar) + m_turretOffset.getY() * Math.cos(thetar) + yr;
    Rotation2d absoluteTurretRotation = getTurretRotation().plus(robotPose.getRotation());
    turretPose = new Pose3d(xt,yt,m_turretOffset.getZ(), new Rotation3d(absoluteTurretRotation));
    Logger.recordOutput("Subsystems/Turret/TurretPose", turretPose);
    ChassisSpeeds robotVelocity = m_robotVelocitySupplier.get();
    turretTranslationalVelocity[0] = (yr-yt)*robotVelocity.omegaRadiansPerSecond + robotVelocity.vxMetersPerSecond;
    turretTranslationalVelocity[1] = (xt - xr)*robotVelocity.omegaRadiansPerSecond + robotVelocity.vyMetersPerSecond;
    Logger.recordOutput("Subsystems/Turret/TurretTranslationalVelocity", turretTranslationalVelocity);
  }

  public Transform3d getTransformToPose(Pose3d target){
    Transform3d out = target.minus(turretPose);
    Logger.recordOutput("Subsystems/Turret/getTransformToPose/Pose", out);
    return out;
  }

  public Pose3d getTurretFieldPose(){
    return turretPose;
  }
  public double[] getTurretTranslationalVelocity(){
    return turretTranslationalVelocity;
  }

  public void setZeroHeading(){
    setEncoder(0.0);
  }

  public void setRotation(Rotation2d rotation){
    double rotationRads = rotation.getRadians();
    rotationRads = MathUtil.clamp(rotationRads, TurretConstants.kTurretMinRotation.get(), TurretConstants.kTurretMaxRotation.get());
    setPosition(rotationRads/TurretConstants.encoderPositionFactor);
  }

  public void pointAt(Translation2d target){
    Logger.recordOutput("Subsystems/Turret/pointAt/target", target);
    Translation2d turretTranslation = new Translation2d(this.turretPose.getX(), this.turretPose.getY());

    Rotation2d targetAngle = target.minus(turretTranslation).getAngle().minus(m_robotPoseSupplier.get().getRotation());
    Logger.recordOutput("Subsystems/Turret/pointAt/targetAngle", targetAngle);
    setRotation(targetAngle);
  
  }
}
