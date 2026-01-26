package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.MultiMotorSubsystem;
import frc.robot.util.motorUtil.RelEncoderFollower;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalOutput;

public class Shooter{
  private final RelEncoderSparkMax leadMotor;
  private final RelEncoderSparkMax followMotor;
  public Shooter() {
    leadMotor = new RelEncoderSparkMax(ShooterConstants.kMotorConfigLead);
    followMotor = new RelEncoderFollower(ShooterConstants.kMotorConfigFollow, leadMotor);
  }

  public void shootVelocity(double velocity) {
    double rpmSetpoint = velocity * ShooterConstants.kShooterVelocityFactor.get();
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/velocity", velocity);
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/rpmSetpoint", rpmSetpoint);
    leadMotor.runFFVelocity(rpmSetpoint);
  }

  

}
