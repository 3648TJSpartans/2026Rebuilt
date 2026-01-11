package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.motorUtil.MotorConfig;

public class TurretConstants {

  public static final MotorConfig kTurretMotorConfig = new MotorConfig("Turret").motorCan(13).p(0.0).d(0.0);
  public static final Pose3d kTurretOffset = new Pose3d(-.20,-0.20,0.20, new Rotation3d());
  
}
