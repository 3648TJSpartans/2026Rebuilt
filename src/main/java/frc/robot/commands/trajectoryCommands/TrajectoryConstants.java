package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;

public class TrajectoryConstants {
  private static final Translation3d blueHubPose = new Translation3d(4.625594, 4.034536, 1.430425);
  private static final Translation3d blueFeedRight = new Translation3d(0.5, 1.5, 0.0);
  private static final Translation3d blueFeedMiddle = new Translation3d(0.5, 4.021, 0.0);
  private static final Translation3d blueFeedLeft = new Translation3d(0.5, 6.542, 0.0);
  public static final TunableNumber overhangHeight =
      new TunableNumber("Trajectory/overhangHeight", 1.5);
  public static final TunableNumber overhangAspect =
      new TunableNumber("Trajectory/overhangAspect", .5);
  public static final Translation3d hubPose = AllianceFlipUtil.apply(blueHubPose);
  public static final Translation3d feedLeft = AllianceFlipUtil.apply(blueFeedLeft);
  public static final Translation3d feedMiddle = AllianceFlipUtil.apply(blueFeedMiddle);
  public static final Translation3d feedRight = AllianceFlipUtil.apply(blueFeedRight);
  public static final double maxTilt = Units.degreesToRadians(7.5);
  public static final TunableNumber preshotDelay =
      new TunableNumber("Trajectory/preshotDelay", 0.0);
  public static final TunableNumber postshotDelay =
      new TunableNumber("Trajectory/postshotDelay", 3.0);
  public static final TunableNumber translationalSpeedThreshold =
      new TunableNumber("Trajectory/translationalSpeedThreshold", 1.0);
  public static final double allianceFeedingCutoffTime = 0.0;
  public static final int movingtargetIts = 2;
  public static final TunableNumber headUpShootSpeed =
      new TunableNumber("Trajectory/headUpShot/shootSpeed", 5.75);
  public static final Rotation2d headUpShotRobotAngle = new Rotation2d(-Math.PI / 2);
  public static final TunableNumber headUpHoodAngle =
      new TunableNumber(
          "Trajectory/headUpShot/shootAngle", Units.degreesToRadians(HoodConstants.minAngle.get()));

  public static final TunableNumber[] rpmMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/0m/RPM", 2000),
        new TunableNumber("Trajectory/.5m/RPM", 2000),
        new TunableNumber("Trajectory/1.0m/RPM", 2250),
        new TunableNumber("Trajectory/1.5m/RPM", 2500),
        new TunableNumber("Trajectory/2.0m/RPM", 3000),
        new TunableNumber("Trajectory/2.5m/RPM", 3200),
        new TunableNumber("Trajectory/3.0m/RPM", 3600),
        new TunableNumber("Trajectory/3.5m/RPM", 3500),
        new TunableNumber("Trajectory/4.0m/RPM", 4000),
        new TunableNumber("Trajectory/4.5m/RPM", 4000),
        new TunableNumber("Trajectory/5.0m/RPM", 4250),
        new TunableNumber("Trajectory/5.5m/RPM", 4500),
        new TunableNumber("Trajectory/6.0m/RPM", 4750),
        new TunableNumber("Trajectory/6.5m/RPM", 5000),
        new TunableNumber("Trajectory/7.0m/RPM", 5250),
        new TunableNumber("Trajectory/7.5m/RPM", 5500),
        new TunableNumber("Trajectory/8.0m/RPM", 5750)
      };

  public static final TunableNumber[] hoodMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/0m/hoodPose", 0.02),
        new TunableNumber("Trajectory/.5m/hoodPose", 0.02),
        new TunableNumber("Trajectory/1.0m/hoodPose", 0.06),
        new TunableNumber("Trajectory/1.5m/hoodPose", 0.10),
        new TunableNumber("Trajectory/2.0m/hoodPose", 0.22),
        new TunableNumber("Trajectory/2.5m/hoodPose", 0.32),
        new TunableNumber("Trajectory/3.0m/hoodPose", 0.38),
        new TunableNumber("Trajectory/3.5m/hoodPose", 0.46),
        new TunableNumber("Trajectory/4.0m/hoodPose", 0.5),
        new TunableNumber("Trajectory/4.5m/hoodPose", 0.57),
        new TunableNumber("Trajectory/5.0m/hoodPose", 0.38),
        new TunableNumber("Trajectory/5.5m/hoodPose", 0.42),
        new TunableNumber("Trajectory/6.0m/hoodPose", 0.46),
        new TunableNumber("Trajectory/6.5m/hoodPose", 0.50),
        new TunableNumber("Trajectory/7.0m/hoodPose", 0.54),
        new TunableNumber("Trajectory/7.5m/hoodPose", 0.58),
        new TunableNumber("Trajectory/8.0m/hoodPose", 0.62)
      };

  public static final TunableNumber[] hangTimeMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/.5m/hangTime", 1.0),
        new TunableNumber("Trajectory/.5m/hangTime", 1.0),
        new TunableNumber("Trajectory/1.0m/hangTime", 1.1),
        new TunableNumber("Trajectory/1.5m/hangTime", 1.2),
        new TunableNumber("Trajectory/2.0m/hangTime", 1.3),
        new TunableNumber("Trajectory/2.5m/hangTime", 1.4),
        new TunableNumber("Trajectory/3.0m/hangTime", 1.5),
        new TunableNumber("Trajectory/3.5m/hangTime", 1.6),
        new TunableNumber("Trajectory/4.0m/hangTime", 1.7),
        new TunableNumber("Trajectory/4.5m/hangTime", 1.8),
        new TunableNumber("Trajectory/5.0m/hangTime", 1.9),
        new TunableNumber("Trajectory/5.5m/hangTime", 2.0),
        new TunableNumber("Trajectory/6.0m/hangTime", 2.1),
        new TunableNumber("Trajectory/6.5m/hangTime", 2.2),
        new TunableNumber("Trajectory/7.0m/hangTime", 2.3),
        new TunableNumber("Trajectory/7.5m/hangTime", 2.4),
        new TunableNumber("Trajectory/8.0m/hangTime", 2.5)
      };

  public static final TunableNumber[] velocityMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/positionAddMatrix/.0m/velocity", -.4),
        new TunableNumber("Trajectory/positionAddMatrix/.5m/velocity", -.4),
        new TunableNumber("Trajectory/positionAddMatrix/1.0m/velocity", -.4),
        new TunableNumber("Trajectory/positionAddMatrix/1.5m/velocity", -.45),
        new TunableNumber("Trajectory/positionAddMatrix/2.0m/velocity", -.55),
        new TunableNumber("Trajectory/positionAddMatrix/2.5m/velocity", -.65),
        new TunableNumber("Trajectory/positionAddMatrix/3.0m/velocity", -.6),
        new TunableNumber("Trajectory/positionAddMatrix/3.5m/velocity", -.7),
        new TunableNumber("Trajectory/positionAddMatrix/4.0m/velocity", -.8),
        new TunableNumber("Trajectory/positionAddMatrix/4.5m/velocity", -.8),
        new TunableNumber("Trajectory/positionAddMatrix/5.0m/velocity", -.75),
        new TunableNumber("Trajectory/positionAddMatrix/5.5m/velocity", -.25),
        new TunableNumber("Trajectory/positionAddMatrix/6.0m/velocity", -.25),
        new TunableNumber("Trajectory/positionAddMatrix/6.5m/velocity", -.25),
        new TunableNumber("Trajectory/positionAddMatrix/7.0m/velocity", -.25),
        new TunableNumber("Trajectory/positionAddMatrix/7.5m/velocity", -.25),
        new TunableNumber("Trajectory/positionAddMatrix/8.0m/velocity", -.25)
      };

  public static final TunableNumber[] turretSpeedMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/SpeedAwayMatrix/-1.0ms/velocity", 0.0),
        new TunableNumber("Trajectory/SpeedAwayMatrix/-0.5ms/velocity", 0.0),
        new TunableNumber("Trajectory/SpeedAwayMatrix/.0ms/velocity", 0.0),
        new TunableNumber("Trajectory/SpeedAwayMatrix/.5ms/velocity", 1.5),
        new TunableNumber("Trajectory/SpeedAwayMatrix/1.0ms/velocity", 2.5)
      };
}
