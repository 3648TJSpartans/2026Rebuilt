package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunDynamicTrajectory extends RunTrajectoryCmd {
  public RunDynamicTrajectory(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Supplier<Translation3d> targetSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> robotTiltSupplier) {
    super(
        turret,
        shooter,
        hood,
        kicker,
        robotPoseSupplier,
        robotTiltSupplier,
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
