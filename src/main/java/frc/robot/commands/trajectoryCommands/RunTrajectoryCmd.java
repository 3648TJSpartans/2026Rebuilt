package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import java.util.function.Supplier;

public class RunTrajectoryCmd extends Command {
  private final Supplier<Trajectory> m_trajectorySupplier;
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Kicker m_kicker;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<Double> m_robotTiltSupplier;

  public RunTrajectoryCmd(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> robotTiltSupplier,
      Supplier<Trajectory> trajectorySupplier) { // TODO include shooter and shooter angle.
    m_trajectorySupplier = trajectorySupplier;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_kicker = kicker;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotTiltSupplier = robotTiltSupplier;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void execute() {
    Trajectory trajectory = m_trajectorySupplier.get();
    // Use the trajectory to control subsystems
    m_turret.setFieldRotation(new Rotation2d(trajectory.getTurretAngle()));
    m_shooter.shootVelocity(trajectory.getShooterSpeed());
    m_hood.setAngle(new Rotation2d(trajectory.getShooterAngle()));
    if (ready()) {
      m_kicker.setSpeed(ShooterConstants.kickerSpeed.get());
    } else {
      m_kicker.setSpeed(ShooterConstants.kickerSlowSpeed.get());
    }
  }

  public boolean ready() {
    return m_turret.positionInTolerance()
        && m_shooter.speedInTolerance()
        && m_hood.positionInTolerance()
        && m_robotTiltSupplier.get() < TrajectoryConstants.maxTilt;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_kicker.stop();
  }
}
