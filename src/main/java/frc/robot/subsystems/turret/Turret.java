package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.motorUtil.RelEncoderSparkMax;

public class Turret extends RelEncoderSparkMax{
  private final Translation3d m_turretOffset;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final Supplier<double[]> m_robotVelocitySupplier;
  private Pose3d turretPose;
  private double[] turretTranslationalVelocity;

  private final DigitalInput m_zeroSwitch;

  public Turret(Supplier<Pose2d> robotPoseSupplier, Supplier<double[]> robotVelocitySupplier) {
    super(TurretConstants.kTurretMotorConfig);
    m_turretOffset = TurretConstants.kTurretOffset;
    m_robotPoseSupplier = robotPoseSupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    turretPose = new Pose3d();
    turretTranslationalVelocity = new double[2];
    m_zeroSwitch = new DigitalInput(TurretConstants.zeroSwitchPort);
  }

  @AutoLogOutput(key = "Subsystems/Turret/TurretAngle")
  public Rotation2d getTurretRotation(){
    return new Rotation2d(getPosition() * TurretConstants.encoderPositionFactor);
  }
  
  @Override
  public void periodic(){
    super.periodic();
    updateInputs();
    checkHeading();
  }

  public void updateInputs(){
    Pose2d robotPose = m_robotPoseSupplier.get();
    double xr = robotPose.getX();
    double yr = robotPose.getY();
    double thetar = robotPose.getRotation().getRadians();
    double xt = m_turretOffset.getX() * Math.cos(thetar) - m_turretOffset.getY() * Math.sin(thetar) + xr;
    double yt = m_turretOffset.getX() * Math.sin(thetar) + m_turretOffset.getY() * Math.cos(thetar) + yr;
    Rotation2d absoluteTurretRotation = getTurretRotation().plus(robotPose.getRotation());
    turretPose = new Pose3d(xt,yt,m_turretOffset.getZ(), new Rotation3d(absoluteTurretRotation));
    Logger.recordOutput("Subsystems/Turret/TurretPose", turretPose);
    double[] robotVelocity = m_robotVelocitySupplier.get();
    turretTranslationalVelocity[0] = (yr-yt)*robotVelocity[2] + robotVelocity[0];
    turretTranslationalVelocity[1] = (xt - xr)*robotVelocity[2] + robotVelocity[1];
    Logger.recordOutput("Subsystems/Turret/TurretTranslationalVelocity", turretTranslationalVelocity);
  }

  public void checkHeading(){
    boolean zeroSwitchState = m_zeroSwitch.get();
    Logger.recordOutput("Subsystems/Turret/ZeroSwitch", zeroSwitchState);
    // TODO this doesn't set zero heading for a >360 turret as it might trigger in multiple poses. If we go that direction, update code. Use floor function as fix. 
    if(zeroSwitchState){
      setZeroHeading();
    }
  }


  public Transform3d getTransformToPose(Pose3d target){
    Transform3d out = target.minus(turretPose);
    Logger.recordOutput("Subsystems/Turret/getTransformToPose/Pose", out);
    return out;
  }

  public Pose3d getTurretFieldPose(){
    return turretPose;
  }
  public double[] getTurretTranslationalVelocity(){
    return turretTranslationalVelocity;
  }

  public void setZeroHeading(){
    setEncoder(0.0);
  }

  public void setRotation(Rotation2d rotation){
    double rotationRads = rotation.getRadians();
    rotationRads = MathUtil.clamp(rotationRads, TurretConstants.kTurretMinRotation.get(), TurretConstants.kTurretMaxRotation.get());
    setPosition(rotationRads/TurretConstants.encoderPositionFactor);
  }

  public void pointAt(Translation2d target){
    Logger.recordOutput("Subsystems/Turret/pointAt/target", target);
    Translation2d turretTranslation = new Translation2d(this.turretPose.getX(), this.turretPose.getY());

    Rotation2d targetAngle = target.minus(turretTranslation).getAngle().minus(m_robotPoseSupplier.get().getRotation());
    Logger.recordOutput("Subsystems/Turret/pointAt/targetAngle", targetAngle);
    setRotation(targetAngle);
  }
}
