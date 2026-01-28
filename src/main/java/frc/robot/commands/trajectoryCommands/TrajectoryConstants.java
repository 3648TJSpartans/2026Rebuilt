package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;

public class TrajectoryConstants {
  public static final Translation3d blueHubPose = new Translation3d(4.625594, 4.034536, 1.430425);
  public static final double overhangHeight = 2.1;
  public static final double overhangAspect = 7.0 / 8;
  public static final Translation3d hubPose = AllianceFlipUtil.apply(blueHubPose);
  public static final double maxTilt = Units.degreesToRadians(7.5);
  public static final double preshotDelay = 0.0;
  public static final double translationalSpeedThreshold = Double.MAX_VALUE;
  public static final double allianceFeedingCutoffTime = 2.5;
}
