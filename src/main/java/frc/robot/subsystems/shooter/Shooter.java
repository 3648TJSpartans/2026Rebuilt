package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RelEncoderSparkMax {
  public Shooter() {
    super(ShooterConstants.kMotorConfig);
  }

  public void shootVelocity(double velocity) {
    double rpmSetpoint = velocity * ShooterConstants.kShooterVelocityFactor.get();
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/velocity", velocity);
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/rpmSetpoint", rpmSetpoint);
    runFFVelocity(rpmSetpoint);
  }
}
