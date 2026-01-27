package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;

public class RangeCalc {
  public static boolean inShootingRange(Pose2d robotPose) {
    return AllianceFlipUtil.apply(robotPose).getX() < PoseConstants.shootXCutoff;
  }
}
