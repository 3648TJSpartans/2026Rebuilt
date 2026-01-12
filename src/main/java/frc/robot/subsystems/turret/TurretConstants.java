package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.motorUtil.MotorConfig;

public class TurretConstants {

  public static final MotorConfig kTurretMotorConfig = new MotorConfig("Subsystems/Turret/MotorIO/").motorCan(13).p(0.0).d(0.0).i(0.0).maxPower(0.1).minPower(0.1);
  public static final Translation3d kTurretOffset = new Translation3d(-.3,.2,.2);
  public static double encoderPositionFactor = 2*Math.PI/5.23;
  
}
