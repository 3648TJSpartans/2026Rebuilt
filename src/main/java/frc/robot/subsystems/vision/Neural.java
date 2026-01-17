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
  }

  public double getTargetRotation() {
    return txnc;
  }

  public Pose2d estimateTargetPose(Drive drive) {
    if (txnc == 0 && tync == 0) {
      return drive.getPose();
    }

    // Get distance
    double distanceToTargetM =
        (VisionConstants.nTargetHeight / 100 - VisionConstants.nCameraHeight / 100)
                / Math.tan(Math.toRadians(tync))
            - 0.5; // Safty adjustment
    Logger.recordOutput("Subsystems/Vision/Neural/distanceToTarget", distanceToTargetM);

    // Calculate transform
    Transform2d transformToTarget =
        new Transform2d(
            new Translation2d(
                distanceToTargetM * Math.cos(Math.toRadians(txnc)),
                distanceToTargetM * Math.sin(Math.toRadians(txnc))),
            new Rotation2d(-Math.toRadians(txnc)));
    Logger.recordOutput("Subsystems/Vision/Neural/transformToTarget", transformToTarget);
    Pose2d targetPose = drive.getPose().transformBy(transformToTarget);
    Logger.recordOutput("Subsystems/Vision/Neural/targetPose", targetPose);
    return targetPose;
  }
}
