package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class Neural extends SubsystemBase {
  private double tx;
  private double ty;
  private double txnc;
  private double tync;

  public Neural(Drive drive) {
    tx = 0;
    ty = 0;
    txnc = 0;
    tync = 0;
  }

  @Override
  public void periodic() {
    tx = LimelightHelpers.getTX("");
    ty = LimelightHelpers.getTY("");
    txnc = LimelightHelpers.getTXNC("");
    tync = LimelightHelpers.getTYNC("");
  }

  public double getTargetRotation() {
    return txnc;
  }

  public Pose2d estimateTargetPose() {
    // Get distance
    double distanceToTargetCm =
        (VisionConstants.nTargetHeight - VisionConstants.nCameraHeight)
            / Math.tan(Math.toRadians(tync));

    // Construct pose
    return new Pose2d(new Translation2d(distanceToTargetCm / 100.0, 0), new Rotation2d(this.txnc));
  }
}
