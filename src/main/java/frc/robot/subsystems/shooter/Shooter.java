package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RelEncoderSparkMax {

  private static RelEncoderSparkMax follower;

  public Shooter() {
    super(ShooterConstants.kLeaderMotorConfig);
    follower = new RelEncoderSparkMax(ShooterConstants.kFollowerMotorConfig);
  }

  public void shootVelocity(double velocity) {
    double rpmSetpoint = velocity * ShooterConstants.kShooterVelocityFactor.get();
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/velocity", velocity);
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/rpmSetpoint", rpmSetpoint);
    runFFVelocity(rpmSetpoint);
  }
}
