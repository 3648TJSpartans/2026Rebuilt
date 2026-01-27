package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shiftTracker.ShiftTracker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunDynamicTrajectory extends RunTrajectoryCmd {

  /*
   * Runs a dynamic trajectory, given a turret, shooter, hood, and kicker. Requires knowledge of the target, if the robot is in range, and how much the robot is tilting.
   * @param turret - Turret Subsystem
   * @param shooter - Shooter Subsystem
   * @param hood - Hood Subsystem
   * @param kicker - Kicker Subsystem
   * @param targetsupplier - Translation3d Supplier for the target (field space)
   * @param inRangeSupplier - Boolean suppleir asking if the robot is in a valid field spot to shoot.
   * @param robotTiltSupplier - Supplys robot tilt... if tilt is too much, don't shoot.
   */
  public RunDynamicTrajectory(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Translation3d> targetSupplier,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier,
      ShiftTracker shiftTracker) {
    super(
        turret,
        shooter,
        hood,
        inRangeSupplier,
        robotTiltSupplier,
        shiftTracker,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          double[] turretVelocity = turret.getTurretTranslationalVelocity();
          Trajectory traj =
              TrajectoryCalc.dynamicTrajectory(
                  turretPose,
                  target,
                  turretVelocity,
                  TrajectoryConstants.overHangAspect,
                  TrajectoryConstants.overhangHeight);
          Logger.recordOutput(
              "Commands/RunDynamicTrajectory/interpolatedTrajectory",
              TrajectoryCalc.interpolateTrajectory(traj, turretVelocity, turretPose));
          return traj;
        });
  }
}
