package frc.robot.commands.trajectoryCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.TrajectoryCalc;

public class RunStationaryTrajectory extends RunTrajectoryCmd {
  public RunStationaryTrajectory(Turret turret, Supplier<Translation3d> targetSupplier) {
    super(turret, () -> {
      Translation3d target = targetSupplier.get();
      Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
      return TrajectoryCalc.stationaryTrajectory(turretPose, target);
    });
  }
  
}
