package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.TunableNumber;
import frc.robot.util.trajectorySolver.MatrixTrajectory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunMatrix extends Command {
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Supplier<Translation3d> m_targetSupplier;
  ;

  public RunMatrix(
      Turret turret, Shooter shooter, Hood hood, Supplier<Translation3d> targetSupplier) {
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_targetSupplier = targetSupplier;
    addRequirements(m_hood, m_shooter, m_turret);
  }

  @Override
  public void execute() {
    MatrixTrajectory trajectory = movingTrajectory();

    m_turret.setFieldRotation(new Rotation2d(trajectory.turretAngle()));
    m_hood.setPosition(trajectory.hoodPose());
    m_shooter.runFFVelocity(trajectory.shooterRPM());
  }

  public boolean isReady() {
    Logger.recordOutput(
        "Commands/RunMatrix/ready/turretPositioned", m_turret.positionInTolerance());
    Logger.recordOutput("Commands/RunMatrix/ready/hoodPositioned", m_hood.positionInTolerance());
    Logger.recordOutput("Commands/RunMatrix/ready/shooterSpeed", m_shooter.speedInTolerance());
    return m_turret.positionInTolerance()
        && m_shooter.speedInTolerance()
        && m_hood.positionInTolerance();
  }

  private MatrixTrajectory stationaryTrajectory() {
    return stationaryTrajectory(
        m_turret.getTurretFieldPose().getTranslation(), m_targetSupplier.get());
  }

  private MatrixTrajectory stationaryTrajectory(Translation3d current, Translation3d target) {
    target = target.minus(current);
    double turretAngle = Math.atan2(target.getY(), target.getX());
    double distance = target.getNorm();
    Logger.recordOutput("Commands/RunMatrix/ready/shotDistance", distance);
    return new MatrixTrajectory(
        linearInterpolate(distance, TrajectoryConstants.hoodMatrix),
        turretAngle,
        linearInterpolate(distance, TrajectoryConstants.rpmMatrix),
        linearInterpolate(distance, TrajectoryConstants.hangTimeMatrix));
  }

  private MatrixTrajectory movingTrajectory(
      Translation3d current, Translation3d target, double[] robotVelocity) {
    MatrixTrajectory trajectory = stationaryTrajectory(current, target);
    for (int i = 0; i < 2; i++) {
      Translation3d newTarget =
          target.minus(
              new Translation3d(
                  robotVelocity[0] * trajectory.hangTime(),
                  robotVelocity[1] * trajectory.hangTime(),
                  0));
      trajectory = stationaryTrajectory(current, newTarget);
    }
    return trajectory;
  }

  private MatrixTrajectory movingTrajectory() {
    return movingTrajectory(
        m_turret.getTurretFieldPose().getTranslation(),
        m_targetSupplier.get(),
        m_turret.getTurretTranslationalVelocity());
  }

  private double linearInterpolate(double distance, TunableNumber[] matrix) {
    int index = (int) (distance / 0.5);
    double remainder = distance % 0.5;
    if (index < 0) {
      index = 0;
    } else if (index >= matrix.length) {
      index = matrix.length - 1;
    }

    double lowerValue = matrix[index].get();
    double upperValue = matrix[Math.min(index + 1, matrix.length - 1)].get();
    return lowerValue + (upperValue - lowerValue) * (remainder / 0.5);
  }
}
