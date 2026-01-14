package frc.robot.commands.trajectoryCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.trajectorySolver.Trajectory;
import java.util.function.Supplier;

public class RunTrajectoryCmd extends Command{
  private final Supplier<Trajectory> m_trajectorySupplier;
  private final Turret m_turret;
  private boolean turretReady = false;
  public RunTrajectoryCmd(Turret turret, Supplier<Trajectory> trajectorySupplier){  //TODO include shooter and shooter angle. 
    m_trajectorySupplier = trajectorySupplier;
    m_turret = turret;
    addRequirements(turret);
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
