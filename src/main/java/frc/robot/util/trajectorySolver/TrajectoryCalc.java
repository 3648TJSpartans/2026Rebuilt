package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class TrajectoryCalc {
  public static final double g = 9.79; // m/s^2
  public static final int movingtargetIts = 1;
  public static final int interpolationPoints = 20;

  public static void main(String[] args) {
    System.out.println("\nMoving Trajectory:");
    Trajectory results =
        dynamicTrajectory(
            new Translation3d(0, 0, 0),
            new Translation3d(4.034536, 4.625594, 1.430425),
            new double[] {-1, 2},
            2);
    System.out.println(results);
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
      Translation3d current, Translation3d target, double thetaShooter) {

    target = target.minus(current);
    Logger.recordOutput(
        "Utils/TrajectoryCalc/Path", new Translation3d[] {current, target.plus(current)});
    double thetaTurret = Math.atan2(target.getY(), target.getX());
    double xpt = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    double zpt = target.getZ();
    double det = 2 * g * (Math.tan(thetaShooter) * xpt - zpt);
    if (det < 0) {
      return Trajectory.invalidTrajectory;
    }
    double shooterSpeed = g * xpt / (Math.cos(thetaShooter) * Math.sqrt(det));
    double hangTime = xpt / (shooterSpeed * Math.cos(thetaShooter));
    return new Trajectory(thetaShooter, thetaTurret, shooterSpeed, hangTime);
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
      Translation3d current, Translation3d target, double[] robotVelocity, double shootAngle) {
    Trajectory trajectory = stationaryTrajectory(current, target, shootAngle);
    for (int i = 0; i < movingtargetIts; i++) {
      Translation3d newTarget =
          target.minus(
              new Translation3d(
                  robotVelocity[0] * trajectory.getHangTime(),
                  robotVelocity[1] * trajectory.getHangTime(),
                  0));
      trajectory = stationaryTrajectory(current, newTarget, shootAngle);
    }
    return trajectory;
  }

  public static Trajectory fixedTimeDynamicTrajectory(
      Translation3d current, Translation3d target, double[] robotVelocity, double hangTime) {
    target = target.minus(current);

    if (hangTime <= 0.0) {
      return Trajectory.invalidTrajectory;
    }
    double yT = target.getY();
    double xT = target.getX();
    double zT = target.getZ();
    double vtx = robotVelocity[0];
    double vty = robotVelocity[1];
    double thetat = Math.atan2(yT / hangTime - vty, xT / hangTime - vtx);
    double thetas =
        Math.atan2((zT / hangTime + g * hangTime / 2.0) * Math.cos(thetat), xT / hangTime - vtx);

    if (thetas <= 0) {
      return Trajectory.invalidTrajectory;
    }
    double vs = (zT / hangTime + g * hangTime / 2.0) / Math.sin(thetas);

    return new Trajectory(thetas, thetat, vs, hangTime);
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

  public static Translation3d[] interpolateTrajectory(
      Trajectory traj, Translation3d turretTranslation) {
    Translation3d[] out = new Translation3d[interpolationPoints];
    double dt = traj.getHangTime() / (interpolationPoints - 1);
    for (int i = 0; i < interpolationPoints; i++) {
      out[i] = trajectoryAtTime(dt * i, traj, new double[] {0.0, 0.0}, turretTranslation);
    }
    return out;
  }

  public static double maxHeight(Trajectory traj) {
    double tmax = traj.getShooterSpeed() * Math.sin(traj.getShooterAngle()) / g;
    return traj.getShooterSpeed() * Math.sin(traj.getShooterAngle()) * tmax - 0.5 * g * tmax * tmax;
  }
}
