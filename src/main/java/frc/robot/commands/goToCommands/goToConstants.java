package frc.robot.commands.goToCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;

public class goToConstants {

        public static final double drivekP = 2.0;
        public static final double drivekD = 0.0;
        public static final double thetakP = 5.0;
        public static final double thetakD = 0.0;
        public static final double driveMaxVelocity = .5;
        public static final double driveMaxAcceleration = 1.5;
        public static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
        public static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
        public static final double driveTolerance = 0.05;
        public static final double thetaTolerance = Units.degreesToRadians(1);
        public static final double ffMinRadius = 0.2;
        public static final double ffMaxRadius = 0.6;

        public static class PoseConstants {
                public final static Pose2d examplePose =
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60));
                public final static Pose2d examplePose2 =
                                new Pose2d(1, 1, Rotation2d.fromDegrees(90));
                public final static double fieldLength = 17.548;
                public final static double fieldWidth = 8.042;

                private static Pose2d[] examplePoseListBlue =
                                new Pose2d[] {new Pose2d(2, 2, Rotation2d.fromDegrees(180)),
                                                new Pose2d(3, 3, Rotation2d.fromDegrees(270))};

                public final static Pose2d[] examplePoseList =
                                AllianceFlipUtil.apply(examplePoseListBlue);

                public static enum AutonState {
                        EXAMPLE1, EXAMPLE2, EXAMPLE3, DEFAULT
                }

        }
}
