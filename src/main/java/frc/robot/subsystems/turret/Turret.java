package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Turret extends RelEncoderSparkMax{
  private final Pose3d m_turretOffset;
  private final Supplier<Pose2d> m_robotPoseSupplier;

  public Turret(Supplier<Pose2d> robotPoseSupplier) {
    super(TurretConstants.kTurretMotorConfig);
    m_turretOffset = TurretConstants.kTurretOffset;
    m_robotPoseSupplier = robotPoseSupplier;
  }

  @AutoLogOutput(key = "Subsystems/Turret/FieldSpacePose")
  public Pose3d getTurretFieldSpace(){
    Pose3d robotPose = new Pose3d(m_robotPoseSupplier.get());
    Translation3d rotatedTranslation = m_turretOffset.getTranslation().rotateAround(new Translation3d(), robotPose.getRotation());
    Logger.recordOutput("Subsystems/Turret/FieldSpacePose/rotatedTranslation",rotatedTranslation);
    return robotPose;
  }

  public Transform3d getTransformToPose(Pose3d target){
    Transform3d out = target.minus(getTurretFieldSpace());
    Logger.recordOutput("Subsystems/Turret/getTransformToPose/Pose", out);
    return out;
  }
}
