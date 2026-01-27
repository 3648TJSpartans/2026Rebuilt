package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shiftTracker.ShiftTracker;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RunTrajectoryCmd extends Command {
  private final Supplier<Trajectory> m_trajectorySupplier;
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Kicker m_kicker;
  private final Supplier<Boolean> m_inRangeSupplier;
  private final Supplier<Double> m_robotTiltSupplier;
  private final ShiftTracker m_shiftTracker;

  public RunTrajectoryCmd(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Supplier<Boolean> inRangeSupplier,
      Supplier<Double> robotTiltSupplier,
      ShiftTracker shiftTracker,
      Supplier<Trajectory> trajectorySupplier) { // TODO include shooter and shooter angle.
    m_trajectorySupplier = trajectorySupplier;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_kicker = kicker;
    m_inRangeSupplier = inRangeSupplier;
    m_robotTiltSupplier = robotTiltSupplier;
    m_shiftTracker = shiftTracker;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void execute() {
    Trajectory trajectory = m_trajectorySupplier.get();
    Logger.recordOutput("Commands/RunTrajectoryCmd/validTrajectory", !trajectory.isValid());
    if (!trajectory.isValid()) {

      return;
    }
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
    boolean offShiftGood =
        !m_shiftTracker.getOnShift()
            && m_trajectorySupplier.get().getHangTime() + TrajectoryConstants.preshotDelay
                < m_shiftTracker.timeUntil();
    /*
     * TODO
     * Theres a world where the shift tracker also rememebrs a 3-second grace period, and keeps shooting after out shift to get balls throughout the grace period.
     */
    boolean timeGood = offShiftGood || m_shiftTracker.getOnShift();
    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/timeGood", timeGood);
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/turretPositioned", m_turret.positionInTolerance());
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/hoodPositioned", m_hood.positionInTolerance());
    Logger.recordOutput(
        "Commands/RunTrajectoryCmd/ready/shooterSpeed", m_shooter.speedInTolerance());
    boolean goodTilt = m_robotTiltSupplier.get() < TrajectoryConstants.maxTilt;

    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/tiltInRange", goodTilt);

    boolean robotInRange = m_inRangeSupplier.get();
    Logger.recordOutput("Commands/RunTrajectoryCmd/ready/robotInRange", robotInRange);
    return m_turret.positionInTolerance()
        && m_shooter.speedInTolerance()
        && m_hood.positionInTolerance()
        && goodTilt
        && robotInRange
        && timeGood;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_kicker.stop();
  }
}
