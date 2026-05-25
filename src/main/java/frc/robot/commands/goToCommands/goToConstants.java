package frc.robot.commands.goToCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableBoolean;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumberAutoUpdater;

public class goToConstants {

  public static final double drivekP = 6.0;
  public static final double drivekI = 0.0;
  public static final double drivekD = 0.02;
  public static final double thetakP = 5.0;
  public static final double thetakI = 0.0;
  public static final double thetakD = 0.0;
  public static final double driveMaxVelocity = 0.25;
  public static final double driveMaxAcceleration = 1.0;
  public static final double thetaMaxVelocity = 360;
  public static final double thetaMaxAcceleration = 720;
  public static final double driveTolerance = 0.005;
  public static final double thetaTolerance = 1; // degree
  public static final double ffMinRadius = 0.2;
  public static final double ffMaxRadius = 0.6;

  public static final TunableBoolean inFieldConstraint =
      new TunableBoolean("Commands/DriveTo/inFieldConstraint", false);

  public static final TunableNumber tunableDriveP =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/P", drivekP, goToConstants::configurePID);
  public static final TunableNumber tunableDriveI =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/I", drivekI, goToConstants::configurePID);
  public static final TunableNumber tunableDriveD =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/D", drivekD, goToConstants::configurePID);
  public static final TunableNumber tunableThetaP =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/P", thetakP, goToConstants::configurePID);
  public static final TunableNumber tunableThetaI =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/I", thetakI, goToConstants::configurePID);
  public static final TunableNumber tunableThetaD =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/D", thetakD, goToConstants::configurePID);
  public static final TunableNumber tunableDriveMaxVelocity =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/maxVelocity", driveMaxVelocity, goToConstants::configurePID);
  public static final TunableNumber tunableDriveMaxAcceleration =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/maxAcceleration",
          driveMaxAcceleration,
          goToConstants::configurePID);
  public static final TunableNumber tunableThetaMaxVelocity =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/maxVelocity(degrees)",
          thetaMaxVelocity,
          goToConstants::configurePID);
  public static final TunableNumber tunableThetaMaxAcceleration =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/maxAcceleration(degrees)",
          thetaMaxAcceleration,
          goToConstants::configurePID);
  public static final TunableNumber tunableDriveTolerance =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/drive/Tolerance", driveTolerance, goToConstants::configurePID);
  public static final TunableNumber tunableThetaTolerance =
      new TunableNumberAutoUpdater(
          "Commands/DriveTo/theta/Tolerance(degrees)", thetaTolerance, goToConstants::configurePID);
  public static ProfiledPIDController driveController;

  public static ProfiledPIDController thetaController;

  public static void configurePID() {
    // Test Print
    // System.out.println("Configuring goToConstants PID controllers with tunable values...");
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
}
