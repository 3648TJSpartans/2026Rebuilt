package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.motorUtil.MotorConfig;

public class TurretConstants {

  public static final MotorConfig kTurretMotorConfig = new MotorConfig("Turret").motorCan(13).p(0.0).d(0.0).i(0.0).maxPower(0.1).minPower(0.1);
  public static final Translation3d kTurretOffset = new Translation3d(.2,-.3,.2);
  public static Rotation2d rotationOffset = new Rotation2d();
  public static double encoderPositionFactor = 1.0;
  
}
