package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.MatrixTrajectory;
import java.util.function.Supplier;

public class RunMatrix extends Command {
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Supplier<Translation3d> m_targetSupplier;
  private final Supplier<Translation3d> m_turretPoseSupplier;

  public RunMatrix(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Translation3d> targetSupplier,
      Supplier<Translation3d> turretPoseSupplier) {
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_targetSupplier = targetSupplier;
    m_turretPoseSupplier = turretPoseSupplier;
  }

  private MatrixTrajectory stationaryTrajectory() {
    Translation3d target =
        m_targetSupplier.get().minus(m_turret.getTurretFieldPose().getTranslation());
    double turretAngle = Math.atan2(target.getY(), target.getX());
    double distance = target.getNorm();

    return new MatrixTrajectory(m_hood.getHoodPose(), turretAngle, shooterRPM(distance), 0);
  }

  private double shooterRPM(double distance) {
    
    return 0; // TODO: implement this
  }
}
