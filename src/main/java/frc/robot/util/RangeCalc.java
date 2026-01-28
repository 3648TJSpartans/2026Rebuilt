package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;

public class RangeCalc {
  public static boolean inShootingRange(Pose2d robotPose) {
    return AllianceFlipUtil.apply(robotPose).getX() < PoseConstants.shootCutoff;
  }

  /*
   * zone 0 is right,
   * zone 1 middle,
   * zone 2 left
   */
  public static int zoneCalc(Pose2d robotPose) {
    double x = robotPose.getX();
    if (x < PoseConstants.zone1) {
      return 0;
    }
    if (x < PoseConstants.zone2) {
      return 1;
    }
    return 2;
  }
}
