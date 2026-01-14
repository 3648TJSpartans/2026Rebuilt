package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;

public class RunStationaryTrajectory extends RunTrajectoryCmd {
  public RunStationaryTrajectory(
      Turret turret, Shooter shooter, Hood hood, Supplier<Translation3d> targetSupplier) {
    super(
        turret,
        shooter,
        hood,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          return TrajectoryCalc.stationaryTrajectory(
              turretPose,
              target,
              TrajectoryConstants.overHangAspect,
              TrajectoryConstants.overhangHeight);
        });
  }
  
}
