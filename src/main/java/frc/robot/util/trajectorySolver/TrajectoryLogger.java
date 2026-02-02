package frc.robot.util.trajectorySolver;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        TrajectoryCalc.interpolateTrajectory(
            measuredTraj, m_turretVelocitySupplier.get(), turretPose));
  }
}
