package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.zoneCalc.Polygon;
import frc.robot.util.zoneCalc.Rectangle;

public class PoseConstants {
  // TODO: update this to an actual pose/poses for climbing
  public static final Pose2d climbPose = new Pose2d(1, 1, Rotation2d.fromDegrees(90));
  public static final double fieldLength = 16.540988;
  public static final double fieldWidth = 8.042;

  public static final double shootCutoff = 4.282694;

  public static final double zone1 = 3;
  public static final double zone2 = 5.042;
  public static final Translation3d feedRight =
      AllianceFlipUtil.apply(new Translation3d(1.0, 1.5, 0.0));
  public static final Translation3d feedMiddle =
      AllianceFlipUtil.apply(new Translation3d(1.0, 4.021, 0.0));
  public static final Translation3d feedLeft =
      AllianceFlipUtil.apply(new Translation3d(1.0, 6.542, 0.0));
  public static final double overhangMiddle = 3.5;
  public static final double overhangSide = 1.5;

  public static final Rectangle test =
      new Rectangle("testZone", new Translation2d(0, 0), new Translation2d(1, 1));
  public static final Polygon testPolygon =
      AllianceFlipUtil.shouldFlip()
          ? AllianceFlipUtil.apply(
              new Polygon(
                  "testPolygon",
                  new Translation2d(0, 0),
                  new Translation2d(2, 4),
                  new Translation2d(4, 0)))
          : AllianceFlipUtil.apply(
              new Polygon(
                  "testPolygon",
                  new Translation2d(0, 0),
                  new Translation2d(2, 4),
                  new Translation2d(4, 0)));
  public static final Polygon behindTheHub =
      new Polygon(
          "BehindTheHub",
          AllianceFlipUtil.apply(
              new Translation2d[] {
                new Translation2d(5.189, 3.23),
                new Translation2d(5.189, 4.83),
                new Translation2d(8.27, 4.034)
              }));
}
