package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;

public class RunDynamicTrajectory extends RunTrajectoryCmd {
  public RunDynamicTrajectory(Turret turret, Supplier<Translation3d> targetSupplier) {
    super(
        turret,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          double[] turretVelocity = turret.getTurretTranslationalVelocity();
          return TrajectoryCalc.dynamicTrajectory(
              turretPose,
              target,
              turretVelocity,
              TrajectoryConstants.overHangAspect,
              TrajectoryConstants.overhangHeight);
        });
  }
  
}
