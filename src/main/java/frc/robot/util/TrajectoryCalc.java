package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;

public class TrajectoryCalc {
  public static final double g = 9.79; // m/s^2
  public static final int movingtargetIts = 5;
  public static final double zOverhang = 3.0;
  public static final double kOverhang = .8;
  public static void main(String[] args) {
    TrajectoryCalc tc = new TrajectoryCalc();
    double[] results = tc.stationaryTrajectory(new Translation3d(0,0,0), new Translation3d(2,2,2));
   
    System.out.println("Stationary Trajectory:");
    for( double i: results){
      System.out.println(i);
    }
    System.out.println("\nMoving Trajectory:");
    results = tc.movingTrajectory(new Translation3d(0,0,0), new Translation3d(2,2,2), new double[]{1,0});
    for( double i: results){
      System.out.println(i);
    }
  }

  /*
   * @returns double[] {shooterAngle,turretAngle,shooterSpeed, hangTime}
   * @link{https://www.desmos.com/3d/f7jbtftncs}
   */
  public double[] stationaryTrajectory(Translation3d current,Translation3d target) {
    double[] out = new double[4];
    Translation3d overhang = new Translation3d(kOverhang*target.getX(), kOverhang*target.getY(), zOverhang);
    Logger.recordOutput("Utils/TrajectoryCalc/Path", new Translation3d[]{current,overhang,target});
    target = target.minus(current);
    overhang = overhang.minus(current);
    double thetaTurret  = Math.atan2(target.getY(), target.getX());
    double xpt = Math.sqrt(target.getX()*target.getX() + target.getY()*target.getY());
    double xpo = Math.sqrt(overhang.getX()*overhang.getX() + overhang.getY()*overhang.getY());
    double zpt = target.getZ();
    double zpo = overhang.getZ();
    double c2 = (zpt*xpo-zpo*xpt)/(xpt*xpt*xpo- xpt*xpo*xpo);
    double c1 = zpo/xpo-c2*xpo;
    // System.out.println("c2 = "+ c2);
    double thetaShooter = Math.atan(c1);
    // System.out.println("T1: "+ Math.sqrt(-g/(2*c2)));
    // System.out.println("T2: "+ Math.cos(thetaShooter));
    double shooterSpeed = Math.sqrt(-g/(2*c2))/Math.cos(thetaShooter);
    double hangTime = xpt/(shooterSpeed* Math.cos(thetaShooter));
    out[0] = thetaShooter;
    out[1] = thetaTurret;
    out[2] = shooterSpeed;
    out[3] = hangTime;
    return out;
  }

  public double[] movingTrajectory(Translation3d current, Translation3d target, double[] robotVelocity){
    double[] trajectory = stationaryTrajectory(current, target);
    for(int i =0; i< movingtargetIts; i++){
      Translation3d newTarget = target.minus(new Translation3d(robotVelocity[0]*trajectory[3], robotVelocity[1]*trajectory[3], 0));
      // System.out.println(newTarget.toString());
      trajectory = stationaryTrajectory(current, newTarget);
    }
    return trajectory;
  }

  public double[] movingTrajectory(Translation3d current, Translation3d target, double[] robotVelocity, double tolerance){
    double[] trajectory = stationaryTrajectory(current, target);
    Translation3d lastTarget = target;
    Translation3d newTarget;
    for(int i =0; i< movingtargetIts; i++){
      newTarget = target.minus(new Translation3d(robotVelocity[0]*trajectory[3], robotVelocity[1]*trajectory[3], 0));
      // System.out.println(newTarget.toString());
      trajectory = stationaryTrajectory(current, newTarget);
      if(lastTarget.minus(newTarget).getNorm() < tolerance){
        System.out.println(i);
        break;
      }
      lastTarget = newTarget;
    }
    return trajectory;
  }
}
