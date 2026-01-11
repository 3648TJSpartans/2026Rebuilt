package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Turret extends RelEncoderSparkMax{
  private final Translation3d m_turretOffset;
  private final Supplier<Pose2d> m_robotPoseSupplier;

  public Turret(Supplier<Pose2d> robotPoseSupplier) {
    super(TurretConstants.kTurretMotorConfig);
    m_turretOffset = TurretConstants.kTurretOffset;
    m_robotPoseSupplier = robotPoseSupplier;
  }

  @AutoLogOutput(key = "Subsystems/Turret/TurretAngle")
  public Rotation2d getTurretRotation(){
    return new Rotation2d(getPosition() * TurretConstants.encoderPositionFactor + TurretConstants.rotationOffset.getRadians());
  }

  @AutoLogOutput(key = "Subsystems/Turret/FieldSpacePose")
  public Pose3d getTurretFieldSpace(){
    Pose2d robotPose = m_robotPoseSupplier.get();
    double xr = robotPose.getX();
    double yr = robotPose.getY();
    double thetar = robotPose.getRotation().getRadians();
    double xt = m_turretOffset.getX() * Math.cos(thetar) - m_turretOffset.getY() * Math.sin(thetar) + xr;
    double yt = m_turretOffset.getX() * Math.sin(thetar) + m_turretOffset.getY() * Math.cos(thetar) + yr;
    Rotation2d absoluteTurretRotation = getTurretRotation().plus(robotPose.getRotation());
    Pose3d turretPose3d = new Pose3d(xt,yt,m_turretOffset.getZ(), new Rotation3d(absoluteTurretRotation));
    return turretPose3d;
  }

  public Transform3d getTransformToPose(Pose3d target){
    Transform3d out = target.minus(getTurretFieldSpace());
    Logger.recordOutput("Subsystems/Turret/getTransformToPose/Pose", out);
    return out;
  }
}
