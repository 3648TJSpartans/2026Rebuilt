package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunTrajectoryCmd extends Command {
  private final Supplier<Trajectory> m_trajectorySupplier;
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Supplier<Boolean> m_inRangeSupplier;
  private final Supplier<Double> m_robotTiltSupplier;
  private Trajectory m_trajectory;

  public RunTrajectoryCmd(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier,
      Supplier<Trajectory> trajectorySupplier) { // TODO include shooter and shooter angle.
    m_trajectorySupplier = trajectorySupplier;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_inRangeSupplier = inRangeSupplier;
    m_robotTiltSupplier = robotTiltSupplier;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void execute() {
    m_trajectory = m_trajectorySupplier.get();
    Logger.recordOutput("Commands/RunTrajectoryCmd/validTrajectory", m_trajectory.isValid());
    if (!m_trajectory.isValid()) {
      return;
    }
    // Use the trajectory to control subsystems
    m_turret.setFieldRotation(new Rotation2d(m_trajectory.getTurretAngle()));
    m_shooter.shootVelocity(m_trajectory.getShooterSpeed());
    m_hood.setAngle(new Rotation2d(m_trajectory.getShooterAngle()));
  }

  public boolean ready() {
    boolean robotInRange = m_inRangeSupplier.get();
    boolean goodTilt = m_robotTiltSupplier.get() < TrajectoryConstants.maxTilt;
    boolean turretTransSpeedGood =
        m_turret.getTurretTranslationalSpeed() < TrajectoryConstants.translationalSpeedThreshold;
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/turretPositioned",
        m_turret.getRelEncoder().positionInTolerance());
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/hoodPositioned", m_hood.positionInTolerance());
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/shooterSpeed", m_shooter.speedInTolerance());
    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/tiltInRange", goodTilt);
    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/robotInRange", robotInRange);
    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/translationalSpeed", turretTransSpeedGood);
    return m_turret.getRelEncoder().positionInTolerance()
        && m_shooter.speedInTolerance()
        && m_hood.positionInTolerance()
        && goodTilt
        && robotInRange
        && turretTransSpeedGood;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_turret.getRelEncoder().stop();
    m_hood.stop();
  }

  public Trajectory getTrajectory() {
    return m_trajectorySupplier.get();
  }
}
