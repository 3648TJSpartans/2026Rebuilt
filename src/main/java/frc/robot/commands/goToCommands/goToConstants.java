package frc.robot.commands.goToCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableBoolean;
import frc.robot.util.miscTunables.TunableProfiledPIDController;

public class goToConstants {

  public static final double drivekP = 6.0;
  public static final double drivekI = 0.0;
  public static final double drivekD = 0.02;
  public static final double thetakP = 5.0;
  public static final double thetakI = 0.0;
  public static final double thetakD = 0.0;
  private static final double driveMaxVelocity = 0.25;
  private static final double driveMaxAcceleration = 1.0;
  private static final double thetaMaxVelocity = Units.degreesToRadians(360);
  private static final double thetaMaxAcceleration = Units.degreesToRadians(720);
  private static final double driveTolerance = 0.005;
  private static final double thetaTolerance = Units.degreesToRadians(1);
  private static final double ffMinRadius = 0.2;
  private static final double ffMaxRadius = 0.6;

  public static final TunableBoolean inFieldConstraint =
      new TunableBoolean("Commands/DriveTo/inFieldConstraint", false);

  private static ProfiledPIDController getDriveController() {
    ProfiledPIDController driveController =
        new ProfiledPIDController(
            drivekP,
            drivekI,
            drivekD,
            new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration),
            0.02);
    driveController.setTolerance(driveTolerance);
    driveController.setGoal(0.0);
    return driveController;
  }

  private static ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            thetakP,
            thetakI,
            thetakD,
            new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration),
            0.02);
    thetaController.setTolerance(thetaTolerance);
    thetaController.setGoal(0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return thetaController;
  }

  public static final TunableProfiledPIDController tunableDriveController =
      new TunableProfiledPIDController(
          "Commands/DriveTo/driveController", goToConstants.getDriveController());
  public static final TunableProfiledPIDController tunableThetaController =
      new TunableProfiledPIDController(
          "Commands/DriveTo/thetaController", goToConstants.getThetaController());
}
