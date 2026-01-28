package frc.robot.commands.goToCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;

public class goToConstants {

  public static final double drivekP = 6.0;
  public static final double drivekI = 0.0;
  public static final double drivekD = 0.02;
  public static final double thetakP = 5.0;
  public static final double thetakI = 0.0;
  public static final double thetakD = 0.0;
  public static final double driveMaxVelocity = 4.0;
  public static final double driveMaxAcceleration = 4.0;
  public static final double thetaMaxVelocity = 360;
  public static final double thetaMaxAcceleration = 720;
  public static final double driveTolerance = 0.005;
  public static final double thetaTolerance = 1; // degree
  public static final double ffMinRadius = 0.2;
  public static final double ffMaxRadius = 0.6;

  public static final TunableNumber tunableDriveP =
      new TunableNumber("Commands/DriveTo/drive/P", drivekP);
  public static final TunableNumber tunableDriveI =
      new TunableNumber("Commands/DriveTo/drive/I", drivekI);
  public static final TunableNumber tunableDriveD =
      new TunableNumber("Commands/DriveTo/drive/D", drivekD);
  public static final TunableNumber tunableThetaP =
      new TunableNumber("Commands/DriveTo/theta/P", thetakP);
  public static final TunableNumber tunableThetaI =
      new TunableNumber("Commands/DriveTo/theta/I", thetakI);
  public static final TunableNumber tunableThetaD =
      new TunableNumber("Commands/DriveTo/theta/D", thetakD);
  public static final TunableNumber tunableDriveMaxVelocity =
      new TunableNumber("Commands/DriveTo/drive/maxVelocity", driveMaxVelocity);
  public static final TunableNumber tunableDriveMaxAcceleration =
      new TunableNumber("Commands/DriveTo/drive/maxAcceleration", driveMaxAcceleration);
  public static final TunableNumber tunableThetaMaxVelocity =
      new TunableNumber("Commands/DriveTo/theta/maxVelocity(degrees)", thetaMaxVelocity);
  public static final TunableNumber tunableThetaMaxAcceleration =
      new TunableNumber("Commands/DriveTo/theta/maxAcceleration(degrees)", thetaMaxAcceleration);
  public static final TunableNumber tunableDriveTolerance =
      new TunableNumber("Commands/DriveTo/drive/Tolerance", driveTolerance);
  public static final TunableNumber tunableThetaTolerance =
      new TunableNumber("Commands/DriveTo/theta/Tolerance(degrees)", thetaTolerance);

  public static ProfiledPIDController driveController;

  public static ProfiledPIDController thetaController;

  public static void configurePID() {
    driveController =
        new ProfiledPIDController(
            tunableDriveP.get(),
            tunableDriveI.get(),
            tunableDriveD.get(),
            new TrapezoidProfile.Constraints(
                tunableDriveMaxVelocity.get(), tunableDriveMaxAcceleration.get()),
            0.02);

    thetaController =
        new ProfiledPIDController(
            tunableThetaP.get(),
            tunableThetaI.get(),
            tunableThetaD.get(),
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(tunableThetaMaxVelocity.get()),
                Units.degreesToRadians(tunableThetaMaxAcceleration.get())),
            0.02);
    driveController.setTolerance(tunableDriveTolerance.get());
    thetaController.setTolerance(Units.degreesToRadians(tunableThetaTolerance.get()));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setGoal(0.0);
    thetaController.setGoal(0.0);
  }

  public static class PoseConstants {
    public static final Pose2d examplePose = new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60));
    public static final Pose2d examplePose2 = new Pose2d(1, 1, Rotation2d.fromDegrees(90));
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.042;

    public static final double shootYCutoff = 4.282694;

    public static final double zone1Y = 3;
    public static final double zone2Y = 5.042;
    public static final Translation3d feedRight = new Translation3d(1.0, 1.5, 0.0);
    public static final Translation3d feedMiddle = new Translation3d(1.0, 4.021, 0.0);
    public static final Translation3d feedLeft = new Translation3d(1.0, 6.542, 0.0);

    private static Pose2d[] examplePoseListBlue =
        new Pose2d[] {
          new Pose2d(2, 2, Rotation2d.fromDegrees(180)),
          new Pose2d(3, 3, Rotation2d.fromDegrees(270))
        };

    public static final Pose2d[] examplePoseList = AllianceFlipUtil.apply(examplePoseListBlue);

    public static enum AutonState {
      EXAMPLE1,
      EXAMPLE2,
      EXAMPLE3,
      DEFAULT
    }
  }
}
