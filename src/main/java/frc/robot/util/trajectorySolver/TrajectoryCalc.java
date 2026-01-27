package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class TrajectoryCalc {
  public static final double g = 9.79; // m/s^2
  public static final int movingtargetIts = 1;
  public static final int interpolationPoints = 20;

  public static void main(String[] args) {
    System.out.println("\nFar Trajectory");
    Trajectory results =
        stationaryTrajectory(
            new Translation3d(0.1778, 0.1778, 0.381),
            new Translation3d(8.255, 0.1778, 0),
            1.0 / 2,
            3.5);
    System.out.println(results);
    System.out.println("\nFar Trajectory, Imperial");
    System.out.println(results.toStringImperial());

    System.out.println("\nClose Trajectory");
    results =
        stationaryTrajectory(
            new Translation3d(3.55, 4.625594, 0.381),
            new Translation3d(4.034536, 4.625594, 1.430425),
            0.5,
            1.6);
    System.out.println(results);
    System.out.println("\nClose Trajectory, Imperial");
    System.out.println(results.toStringImperial());
  }

  /*
   * @returns double[] {shooterAngle,turretAngle,shooterSpeed, hangTime}
   * @link{https://www.desmos.com/3d/f7jbtftncs}
   */
  public static Trajectory stationaryTrajectory(
      Translation3d current, Translation3d target, double overhangRatio, double zOverhang) {

    target = target.minus(current);
    Translation3d overhang =
        new Translation3d(overhangRatio * target.getX(), overhangRatio * target.getY(), zOverhang);
    // overhang = overhang;
    Logger.recordOutput(
        "Utils/TrajectoryCalc/Path",
        new Translation3d[] {current, overhang.plus(current), target.plus(current)});
    double thetaTurret = Math.atan2(target.getY(), target.getX());
    double xpt = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    double xpo = Math.sqrt(overhang.getX() * overhang.getX() + overhang.getY() * overhang.getY());
    double zpt = target.getZ();
    double zpo = overhang.getZ();
    double c2 = (zpt * xpo - zpo * xpt) / (xpt * xpt * xpo - xpt * xpo * xpo);
    double c1 = zpo / xpo - c2 * xpo;
    double thetaShooter = Math.atan(c1);
    double shooterSpeed = Math.sqrt(-g / (2 * c2)) / Math.cos(thetaShooter);
    double hangTime = xpt / (shooterSpeed * Math.cos(thetaShooter));
    return new Trajectory(thetaShooter, thetaTurret, shooterSpeed, hangTime);
  }

  public static Trajectory stationaryTrajectory(
      Translation3d current, Translation3d target, double shooterAngle) {
    target = target.minus(current);
    // overhang = overhang;
    Logger.recordOutput(
        "Utils/TrajectoryCalc/Path", new Translation3d[] {current, target.plus(current)});
    double thetaTurret = Math.atan2(target.getY(), target.getX());
    double xpt = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    double zpt = target.getZ();
    double det = 2 * g * (Math.tan(shooterAngle) * xpt - zpt);
    if (det < 0) {
      return new Trajectory(0.0, 0.0, 0.0, 0.0);
    }
    double shooterSpeed = g * xpt / (Math.cos(shooterAngle) * Math.sqrt(det));
    double hangTime = xpt / (shooterSpeed * Math.cos(shooterAngle));
    return new Trajectory(shooterAngle, thetaTurret, shooterSpeed, hangTime);
  }

  public static Trajectory dynamicTrajectory(
      Translation3d current,
      Translation3d target,
      double[] robotVelocity,
      double overhangRatio,
      double zOverhang) {
    Trajectory trajectory = stationaryTrajectory(current, target, overhangRatio, zOverhang);
    for (int i = 0; i < movingtargetIts; i++) {
      Translation3d newTarget =
          target.minus(
              new Translation3d(
                  robotVelocity[0] * trajectory.getHangTime(),
                  robotVelocity[1] * trajectory.getHangTime(),
                  0));
      trajectory = stationaryTrajectory(current, newTarget, overhangRatio, zOverhang);
    }
    return trajectory;
  }

  public static Trajectory dynamicTrajectory(
      Translation3d current, Translation3d target, double[] robotVelocity, double shooterAngle) {
    Trajectory trajectory = stationaryTrajectory(current, target, shooterAngle);
    for (int i = 0; i < movingtargetIts; i++) {
      Translation3d newTarget =
          target.minus(
              new Translation3d(
                  robotVelocity[0] * trajectory.getHangTime(),
                  robotVelocity[1] * trajectory.getHangTime(),
                  0));
      trajectory = stationaryTrajectory(current, newTarget, shooterAngle);
    }
    return trajectory;
  }

  public static Translation3d trajectoryAtTime(
      double time, Trajectory traj, double[] turretVelocity, Translation3d turretTranslation) {
    return new Translation3d(
            (traj.getShooterSpeed()
                        * Math.cos(traj.getShooterAngle())
                        * Math.cos(traj.getTurretAngle())
                    + turretVelocity[0])
                * time,
            (traj.getShooterSpeed()
                        * Math.cos(traj.getShooterAngle())
                        * Math.sin(traj.getTurretAngle())
                    + turretVelocity[1])
                * time,
            traj.getShooterSpeed() * Math.sin(traj.getShooterAngle()) * time - g * time * time / 2)
        .plus(turretTranslation);
  }

  public static Translation3d[] interpolateTrajectory(
      Trajectory traj, double[] turretVelocity, Translation3d turretTranslation) {
    Translation3d[] out = new Translation3d[interpolationPoints];
    double dt = traj.getHangTime() / (interpolationPoints - 1);
    for (int i = 0; i < interpolationPoints; i++) {
      out[i] = trajectoryAtTime(dt * i, traj, turretVelocity, turretTranslation);
    }
    return out;
  }
}
