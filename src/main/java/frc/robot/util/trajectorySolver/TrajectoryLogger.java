package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrajectoryLogger extends SubsystemBase {
  private final DoubleSupplier m_shooterAngleSupplier;
  private final DoubleSupplier m_turretAngleSupplier;
  private final DoubleSupplier m_shooterSpeedSupplier;
  private final DoubleSupplier m_hangTimeSupplier;
  private final Supplier<Translation3d> m_turretPoseSupplier;
  private final Supplier<double[]> m_turretVelocitySupplier;
  private final ArrayList<Ball> balls;
  private double secondCounter = 0;

  private static final double SECONDS_PER_BALL = 1.0 / 3.0;

  public TrajectoryLogger(
      DoubleSupplier shooterAngleSupplier,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier shooterSpeedSupplier,
      DoubleSupplier hangTimeSupplier,
      Supplier<Translation3d> turretPoseSupplier,
      Supplier<double[]> turretVelocitySupplier) {
    m_hangTimeSupplier = hangTimeSupplier;
    m_shooterAngleSupplier = shooterAngleSupplier;
    m_shooterSpeedSupplier = shooterSpeedSupplier;
    m_turretAngleSupplier = turretAngleSupplier;
    m_turretPoseSupplier = turretPoseSupplier;
    m_turretVelocitySupplier = turretVelocitySupplier;
    balls = new ArrayList<>();
  }

  @Override
  public void periodic() {

    Trajectory measuredTraj =
        new Trajectory(
            m_shooterAngleSupplier.getAsDouble(),
            m_turretAngleSupplier.getAsDouble(),
            m_shooterSpeedSupplier.getAsDouble(),
            m_hangTimeSupplier.getAsDouble());

    Translation3d turretPose = m_turretPoseSupplier.get();
    double[] turretVelocity = m_turretVelocitySupplier.get();
    updateBalls(measuredTraj, turretPose, turretVelocity);
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/hangTime", measuredTraj.getHangTime());
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/shooterSpeed",
        measuredTraj.getShooterSpeed());
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/shooterAngle",
        measuredTraj.getShooterAngle());
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/turretRotation",
        measuredTraj.getTurretRotation());
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/shooterPose",
        new Pose3d(
            turretPose,
            new Rotation3d(0, -measuredTraj.getShooterAngle(), measuredTraj.getTurretAngle())));
    Logger.recordOutput(
        "Utils/TrajectoryLogger/Measured/measuredTrajectory/path",
        TrajectoryCalc.interpolateTrajectory(measuredTraj, turretVelocity, turretPose));
  }

  public void updateBalls(
      Trajectory measuredTraj, Translation3d turretPose, double[] turretVelocity) {
    secondCounter += 0.02;
    if (secondCounter > SECONDS_PER_BALL) {
      secondCounter = 0;
      balls.add(
          new Ball(
              measuredTraj.copy(),
              Arrays.copyOf(turretVelocity, 2),
              new Translation3d(turretPose.getX(), turretPose.getY(), turretPose.getZ())));
    }
    balls.removeIf(ball -> ball.isDead());
    balls.forEach(Ball::update);
    Logger.recordOutput(
        "Utils/TrajectoryLogger/ballPoses",
        balls.stream().map(Ball::getTranslation).toArray(Translation3d[]::new));
  }
}
