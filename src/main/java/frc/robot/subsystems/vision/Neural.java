package frc.robot.subsystems.vision;

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
}
