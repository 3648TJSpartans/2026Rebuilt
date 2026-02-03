package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Neural extends SubsystemBase {
  private double tx;
  private double ty;
  private double txnc;
  private double tync;
  private boolean targetDetected;
  private Pose2d targetPose;
  private Pose2d savedPose;
  private final Supplier<Pose2d> robotPoseSupplier;

  public Neural(Supplier<Pose2d> robotPoseSupplier) {
    tx = 0;
    ty = 0;
    txnc = 0;
    tync = 0;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    tx = -LimelightHelpers.getTX(VisionConstants.camera0Name);
    ty = -LimelightHelpers.getTY(VisionConstants.camera0Name);
    txnc = -LimelightHelpers.getTXNC(VisionConstants.camera0Name);
    tync = -LimelightHelpers.getTYNC(VisionConstants.camera0Name);
    Logger.recordOutput("Subsystems/Vision/Neural/tx", tx);
    Logger.recordOutput("Subsystems/Vision/Neural/ty", ty);
    Logger.recordOutput("Subsystems/Vision/Neural/txnc", txnc);
    Logger.recordOutput("Subsystems/Vision/Neural/tync", tync);
    targetDetected = tx != 0 && ty != 0;
    Logger.recordOutput("Subsystems/Vision/Neural/targetDetected", targetDetected);
    estimateTargetPose(robotPoseSupplier.get());
  }

  // We already have a method to switch pipelines, but I'm not sure how to integrate the limelight
  // index system with this code.
  public void namedSwitchPipeline(String cameraName, int pipeline) {
    LimelightHelpers.setPipelineIndex(cameraName, pipeline);
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  public void updateSavedPose() {
    savedPose = targetPose;
  }

  public boolean isPoseDetected() {
    return targetDetected;
  }

  public double getTargetRotation() {
    return -tx;
  }

  public double getTargetVertAngle() {
    return ty;
  }

  public void estimateTargetPose(Pose2d robotPose) {
    if (tx == 0 && ty == 0) {
      return;
    } else {

      // Get distance
      double distanceToTargetM =
          (VisionConstants.nTargetHeight / 100 - VisionConstants.nCameraHeight / 100)
              / Math.tan(Math.toRadians(ty));
      Logger.recordOutput("Subsystems/Vision/Neural/distanceToTarget", distanceToTargetM);

      // Calculate transform
      Transform2d transformToTarget =
          new Transform2d(
              new Translation2d(
                  distanceToTargetM * Math.cos(Math.toRadians(tx)),
                  -(distanceToTargetM * Math.sin(Math.toRadians(tx)))),
              new Rotation2d());
      Logger.recordOutput("Subsystems/Vision/Neural/transformToTarget", transformToTarget);
      Pose2d targetPose = robotPose.transformBy(transformToTarget);
      Logger.recordOutput("Subsystems/Vision/Neural/targetPose", targetPose);
      this.targetPose = targetPose;
    }
  }
}
