package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunDynamicTrajectory extends RunTrajectoryCmd {

  /*
   * Runs a dynamic trajectory, given a turret, shooter, hood, and kicker.
   * Requires knowledge of the target, if the robot is in range, and how much the
   * robot is tilting.
   *
   * @param turret - Turret Subsystem
   *
   * @param shooter - Shooter Subsystem
   *
   * @param hood - Hood Subsystem
   *
   * @param kicker - Kicker Subsystem
   *
   * @param targetsupplier - Translation3d Supplier for the target (field space)
   *
   * @param inRangeSupplier - Boolean suppleir asking if the robot is in a valid
   * field spot to shoot.
   *
   * @param robotTiltSupplier - Supplys robot tilt... if tilt is too much, don't
   * shoot.
   */
  public RunDynamicTrajectory(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Double> overhangHeight,
      Supplier<Double> overhangAspect,
      Supplier<Translation3d> targetSupplier,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier) {
    super(
        turret,
        shooter,
        hood,
        inRangeSupplier,
        robotTiltSupplier,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          double[] turretVelocity = turret.getTurretTranslationalVelocity();
          Trajectory traj =
              TrajectoryCalc.dynamicTrajectory(
                  turretPose, target, turretVelocity, overhangAspect.get(), overhangHeight.get());
          if (traj.getShooterAngle() < Units.degreesToRadians(HoodConstants.maxAngle.get())) {
            traj =
                TrajectoryCalc.dynamicTrajectory(
                    turretPose,
                    target,
                    turretVelocity,
                    Units.degreesToRadians(HoodConstants.maxAngle.get()));
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", true);
          } else if (traj.getShooterAngle()
              > Units.degreesToRadians(HoodConstants.minAngle.get())) {
            traj =
                TrajectoryCalc.dynamicTrajectory(
                    turretPose,
                    target,
                    turretVelocity,
                    Units.degreesToRadians(HoodConstants.minAngle.get()));
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", true);
          } else {
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", false);
          }
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/hangTime", traj.getHangTime());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterSpeed", traj.getShooterSpeed());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterAngle", traj.getShooterAngle());
            Logger.recordOutput(
              "Commands/RunDynamicMatrixTrajectory/trajectory/error",
              traj.getError());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/turretRotation", traj.getTurretRotation());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterPose",
              new Pose3d(
                  turretPose, new Rotation3d(0, -traj.getShooterAngle(), traj.getTurretAngle())));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/maxHeight", TrajectoryCalc.maxHeight(traj));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/interpolatedTrajectory",
              TrajectoryCalc.interpolateTrajectory(traj, turretVelocity, turretPose));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/interpolatedTrajectory (no velocity)",
              TrajectoryCalc.interpolateTrajectory(traj, turretPose));
          return traj;
        });
  }

  public RunDynamicTrajectory(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Double> overhangHeight,
      Supplier<Double> overhangAspect,
      Supplier<Translation3d> targetSupplier,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier,
      boolean turretBroken,
      boolean hoodBroken) {
    super(
        turret,
        shooter,
        hood,
        inRangeSupplier,
        robotTiltSupplier,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          double[] turretVelocity = turret.getTurretTranslationalVelocity();
          Trajectory traj =
              TrajectoryCalc.dynamicTrajectory(
                  turretPose, target, turretVelocity, overhangAspect.get(), overhangHeight.get());
          if (hoodBroken) {
            traj =
                TrajectoryCalc.dynamicTrajectory(
                    turretPose,
                    target,
                    turretVelocity,
                    Units.degreesToRadians(HoodConstants.minAngle.get()));
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/hoodBroken", true);
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", true);
          } else if (traj.getShooterAngle()
              < Units.degreesToRadians(HoodConstants.maxAngle.get())) {
            traj =
                TrajectoryCalc.dynamicTrajectory(
                    turretPose,
                    target,
                    turretVelocity,
                    Units.degreesToRadians(HoodConstants.maxAngle.get()));
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", true);
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/hoodBroken", false);
          } else if (traj.getShooterAngle()
              > Units.degreesToRadians(HoodConstants.minAngle.get())) {
            traj =
                TrajectoryCalc.dynamicTrajectory(
                    turretPose,
                    target,
                    turretVelocity,
                    Units.degreesToRadians(HoodConstants.minAngle.get()));
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", true);
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/hoodBroken", false);
          } else {
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/endCapped", false);
            Logger.recordOutput("Commands/RunDynamicTrajectory/trajectory/hoodBroken", false);
          }
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/hangTime", traj.getHangTime());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterSpeed", traj.getShooterSpeed());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterAngle", traj.getShooterAngle());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/turretRotation", traj.getTurretRotation());
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/trajectory/shooterPose",
              new Pose3d(
                  turretPose, new Rotation3d(0, -traj.getShooterAngle(), traj.getTurretAngle())));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/maxHeight", TrajectoryCalc.maxHeight(traj));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/interpolatedTrajectory",
              TrajectoryCalc.interpolateTrajectory(traj, turretVelocity, turretPose));
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/interpolatedTrajectory (no velocity)",
              TrajectoryCalc.interpolateTrajectory(traj, turretPose));
          return traj;
        },
        turretBroken,
        hoodBroken);
  }
}
