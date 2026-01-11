package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    Pose3d turretPose = robotPose.plus(new Transform3d(new Pose3d(), m_turretOffset.rotateBy(robotPose.getRotation())));
    return turretPose;
  }

  public Transform3d getTransformToPose(Pose3d target){
    return target.minus(getTurretFieldSpace());
  }
}
