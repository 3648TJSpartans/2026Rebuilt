package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Translation3d;

public class Ball {
  private Translation3d translation;
  private final double[] turretVelocity;
  private final Trajectory traj;
  private final Translation3d turretTranslation;
  private double time;

  public Ball(Trajectory traj, double[] turretVelocity, Translation3d turretTranslation) {
    time = 0.0;
    this.traj = traj;
    this.turretVelocity = turretVelocity;
    this.turretTranslation = turretTranslation;
    translation = TrajectoryCalc.trajectoryAtTime(0, traj, turretVelocity, turretTranslation);
  }

  public void update() {
    time += 0.02;
    translation = TrajectoryCalc.trajectoryAtTime(time, traj, turretVelocity, turretTranslation);
  }

  public Translation3d getTranslation() {
    return translation;
  }

  public boolean isDead() {
    if (getTranslation().getZ() < 0) {
      return true;
    }
    return false;
  }
}
