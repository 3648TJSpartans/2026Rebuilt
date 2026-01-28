package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;

public class RangeCalc {
  public static boolean inShootingRange(Pose2d robotPose) {
    return AllianceFlipUtil.apply(robotPose).getY() < PoseConstants.shootYCutoff;
  }

  /*
   * zone 0 is right,
   * zone 1 middle,
   * zone 2 left
   */
  public int zoneCalc(Pose2d robotPose) {
    double y = robotPose.getY();
    if (y < PoseConstants.zone1Y) {
      return 0;
    }
    if (y < PoseConstants.zone2Y) {
      return 1;
    }
    return 2;
  }
}
