package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class Neural extends SubsystemBase {
  private double tx;
  private double ty;
  private double txnc;
  private double tync;
  private Pose2d targetPose;

  public Neural() {
    tx = 0;
    ty = 0;
    txnc = 0;
    tync = 0;
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
    Logger.recordOutput("Subsystems/Vision/Neural/targetDetected", tx != 0 && ty != 0);
  }

  // We already have a method to switch pipelines, but I'm not sure how to integrate the limelight
  // index system with this code.
  public void namedSwitchPipeline(String cameraName, int pipeline) {
    LimelightHelpers.setPipelineIndex(cameraName, pipeline);
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public double getTargetRotation() {
    return -tx;
  }

  public double getTargetVertAngle() {
    return ty;
  }

  public void estimateTargetPose(Drive drive) {
    System.out.println("Neural pose estimator has run!");
    if (tx == 0 && ty == 0) {
      this.targetPose = drive.getPose();
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
                  distanceToTargetM * Math.sin(Math.toRadians(tx))),
              new Rotation2d(-Math.toRadians(tx)));
      Logger.recordOutput("Subsystems/Vision/Neural/transformToTarget", transformToTarget);
      Pose2d targetPose = drive.getPose().transformBy(transformToTarget);
      Logger.recordOutput("Subsystems/Vision/Neural/targetPose", targetPose);
      this.targetPose = targetPose;
    }
  }
}
