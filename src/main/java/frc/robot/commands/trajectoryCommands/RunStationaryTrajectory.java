package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.TrajectoryCalc;
import java.util.function.Supplier;

public class RunStationaryTrajectory extends RunTrajectoryCmd {
  public RunStationaryTrajectory(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Double> overhangHeight,
      Supplier<Double> overhangAspect,
      Supplier<Translation3d> targetSupplier,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier,
      Supplier<Double> timeLeft,
      Supplier<Double> timeTill) {
    super(
        turret,
        shooter,
        hood,
        inRangeSupplier,
        robotTiltSupplier,
        timeLeft,
        timeTill,
        () -> {
          Translation3d target = targetSupplier.get();
          Translation3d turretPose = turret.getTurretFieldPose().getTranslation();
          return TrajectoryCalc.stationaryTrajectory(
              turretPose, target, overhangAspect.get(), overhangHeight.get());
        });
  }
}
