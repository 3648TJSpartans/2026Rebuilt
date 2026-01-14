package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import java.util.function.Supplier;

public class RunTrajectoryCmd extends Command {
  private final Supplier<Trajectory> m_trajectorySupplier;
  private final Turret m_turret;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private boolean turretReady = false;

  public RunTrajectoryCmd(
      Turret turret,
      Shooter shooter,
      Hood hood,
      Supplier<Trajectory> trajectorySupplier) { // TODO include shooter and shooter angle.
    m_trajectorySupplier = trajectorySupplier;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void execute() {
    Trajectory trajectory = m_trajectorySupplier.get();
    // Use the trajectory to control subsystems
    m_turret.setFieldRotation(new Rotation2d(trajectory.getTurretAngle()));

  }

  public boolean ready(){
    return m_turret.positionInTolerance();
  }
} 
