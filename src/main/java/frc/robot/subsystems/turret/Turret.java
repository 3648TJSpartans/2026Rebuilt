package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderIO;
import frc.robot.util.statusableUtils.Statusable;
import frc.robot.util.statusableUtils.StatusableDigitalInput;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements Statusable {
  private final Translation3d m_turretOffset;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<double[]> m_robotVelocitySupplier;
  private Pose3d turretPose;
  private double[] turretTranslationalVelocity;
  private final StatusableDigitalInput m_zeroSwitch;
  private boolean isHomed;
  private RelEncoderIO m_relEncoder;

  public Turret(
      RelEncoderIO relEncoder,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<double[]> robotVelocitySupplier) {
    m_relEncoder = relEncoder;
    m_turretOffset = TurretConstants.kTurretOffset;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    turretPose = new Pose3d();
    turretTranslationalVelocity = new double[2];
    m_zeroSwitch =
        new StatusableDigitalInput(TurretConstants.zeroSwitchPort, "Subsystems/Turret/ZeroSwitch");
    isHomed = false;
  }

  @AutoLogOutput(key = "Subsystems/Turret/TurretAngle")
  public Rotation2d getTurretRotation() {
    return new Rotation2d(m_relEncoder.getPosition() * TurretConstants.encoderPositionFactor);
  }

  @Override
  public void periodic() {
    m_relEncoder.periodic();
    updateInputs();
    checkHeading();
  }

  public void updateInputs() {
    Pose2d robotPose = m_robotPoseSupplier.get();
    double xr = robotPose.getX();
    double yr = robotPose.getY();
    double thetar = robotPose.getRotation().getRadians();
    double xt =
        m_turretOffset.getX() * Math.cos(thetar) - m_turretOffset.getY() * Math.sin(thetar) + xr;
    double yt =
        m_turretOffset.getX() * Math.sin(thetar) + m_turretOffset.getY() * Math.cos(thetar) + yr;
    Rotation2d absoluteTurretRotation = getTurretRotation().plus(robotPose.getRotation());
    turretPose = new Pose3d(xt, yt, m_turretOffset.getZ(), new Rotation3d(absoluteTurretRotation));
    Logger.recordOutput("Subsystems/Turret/TurretPose", turretPose);
    double[] robotVelocity = m_robotVelocitySupplier.get();
    turretTranslationalVelocity[0] = (yr - yt) * robotVelocity[2] + robotVelocity[0];
    turretTranslationalVelocity[1] = (xt - xr) * robotVelocity[2] + robotVelocity[1];
    Logger.recordOutput(
        "Subsystems/Turret/TurretTranslationalVelocity", turretTranslationalVelocity);
  }

  public void checkHeading() {
    boolean zeroSwitchState = m_zeroSwitch.get();
    Logger.recordOutput("Subsystems/Turret/ZeroSwitch/Pushed", zeroSwitchState);
    // TODO this doesn't set zero heading for a >360 turret as it might trigger in multiple poses.
    // If we go that direction, update code. Use floor function as fix.
    if (zeroSwitchState) {
      // Allows us to rotate turret 360 degrees and get our encoder offset value.
      Logger.recordOutput("Subsystems/Turret/ZeroSwitch/delta", m_relEncoder.getPosition());
      setZeroHeading();
    }
  }

  public Transform3d getTransformToPose(Pose3d target) {
    Transform3d out = target.minus(turretPose);
    Logger.recordOutput("Subsystems/Turret/getTransformToPose/Pose", out);
    return out;
  }

  public Pose3d getTurretFieldPose() {
    return turretPose;
  }

  public double[] getTurretTranslationalVelocity() {
    return turretTranslationalVelocity;
  }

  public double getTurretTranslationalSpeed() {
    double[] velocity = getTurretTranslationalVelocity();
    return Math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
  }

  public void setZeroHeading() {
    isHomed = true;
    m_relEncoder.setEncoder(TurretConstants.turretZeroingOffset.get());
  }

  public RelEncoderIO getRelEncoder() {
    return m_relEncoder;
  }

  // sets rotation in robot space
  public void setRotation(Rotation2d rotation) {
    if (!isHomed) {
      return;
    }
    double rotationRads = rotation.getRadians();
    rotationRads =
        MathUtil.clamp(
            rotationRads,
            TurretConstants.kTurretMinRotation.get(),
            TurretConstants.kTurretMaxRotation.get());
    m_relEncoder.setPosition(rotationRads / TurretConstants.encoderPositionFactor);
  }

  // Sets the roation in field space
  public void setFieldRotation(Rotation2d rotation) {
    Rotation2d robotRotation = m_robotPoseSupplier.get().getRotation();
    Rotation2d turretRotation = rotation.minus(robotRotation);
    setRotation(turretRotation);
  }

  public void pointAt(Translation2d feedmiddle) {
    Logger.recordOutput("Subsystems/Turret/pointAt/target", feedmiddle);
    Translation2d turretTranslation =
        new Translation2d(this.turretPose.getX(), this.turretPose.getY());

    Rotation2d targetAngle =
        feedmiddle
            .minus(turretTranslation)
            .getAngle()
            .minus(m_robotPoseSupplier.get().getRotation());
    Logger.recordOutput("Subsystems/Turret/pointAt/targetAngle", targetAngle);
    setRotation(targetAngle);
  }

  @Override
  public Status getStatus() {
    if (m_relEncoder.getStatus() != Status.OK) {
      Logger.recordOutput("Debug/Subsystems/Turret/error", "Motor not attatched");
      return m_relEncoder.getStatus();
    }
    if (!isHomed) {
      Logger.recordOutput("Debug/Subsystems/Turret/warning", "Not Homed");
      return Status.WARNING;
    }
    if (m_zeroSwitch.getStatus() != Status.OK) {
      Logger.recordOutput("Debug/Subsystems/Turret/warning", "Limit Switch");
      return m_zeroSwitch.getStatus();
    }
    return Status.OK;
  }

  @AutoLogOutput(key = "Subsystems/Turret/homed")
  public boolean getHomed() {
    return isHomed;
  }

  @Override
  public String getName() {
    return "Subsystems/Turret";
  }
}
