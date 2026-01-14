package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AllianceFlipUtil;

public class TrajectoryConstants {
  public static Translation3d blueHubPose = new Translation3d(4.034536, 182.11, 1.430425);
  public static double overhangHeight = 2.1336;
  public static double overHangAspect = 7 / 8;
  public static Translation3d hubPose = AllianceFlipUtil.apply(blueHubPose);
}
