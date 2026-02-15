package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;

public class TrajectoryConstants {
  public static final Translation3d blueHubPose = new Translation3d(4.625594, 4.034536, 1.430425);
  public static final TunableNumber overhangHeight =
      new TunableNumber("Trajectory/overhangHeight", 2.1);
  public static final TunableNumber overhangAspect =
      new TunableNumber("Trajectory/overhangAspect", 7.0 / 8);
  public static final Translation3d hubPose = AllianceFlipUtil.apply(blueHubPose);
  public static final double maxTilt = Units.degreesToRadians(7.5);
  public static final double preshotDelay = 0.0;
  public static final double translationalSpeedThreshold = Double.MAX_VALUE;
  public static final double allianceFeedingCutoffTime = 0.0;

  public static TunableNumber[] rpmMatrix =
      new TunableNumber[] {
        new TunableNumber("Trajectory/RPM/.5m", 2000),
        new TunableNumber("Trajectory/RPM/1.0m", 2250),
        new TunableNumber("Trajectory/RPM/1.5m", 2500),
        new TunableNumber("Trajectory/RPM/2.0m", 2750),
        new TunableNumber("Trajectory/RPM/2.5m", 3000),
        new TunableNumber("Trajectory/RPM/3.0m", 3250),
        new TunableNumber("Trajectory/RPM/3.5m", 3500),
        new TunableNumber("Trajectory/RPM/4.0m", 3750),
        new TunableNumber("Trajectory/RPM/4.5m", 4000),
        new TunableNumber("Trajectory/RPM/5.0m", 4250),
        new TunableNumber("Trajectory/RPM/5.5m", 4500),
        new TunableNumber("Trajectory/RPM/6.0m", 4750),
        new TunableNumber("Trajectory/RPM/6.5m", 5000),
        new TunableNumber("Trajectory/RPM/7.0m", 5250),
        new TunableNumber("Trajectory/RPM/7.5m", 5500),
        new TunableNumber("Trajectory/RPM/8.0m", 5750),
      };
}
