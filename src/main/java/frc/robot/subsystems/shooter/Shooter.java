package frc.robot.subsystems.shooter;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RelEncoderSparkMax {

  private final RelEncoderSparkMax follower;

  public Shooter() {
    super(ShooterConstants.kLeaderMotorConfig);
    follower = new RelEncoderSparkMax(ShooterConstants.kFollowerMotorConfig);
  }

  @Override
  public void setPower(double power) {
    super.setPower(power);
    follower.setPower(-power);
  }

  public void shootVelocity(double velocity) {
    double rpmSetpoint =
        velocity * ShooterConstants.kShooterVelocityFactor.get()
            + ShooterConstants.rpmThreshold.get();
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/velocity", velocity);
    Logger.recordOutput("Subsystems/Shooter/shootVelocity/rpmSetpoint", rpmSetpoint);
    runFFVelocity(rpmSetpoint);
  }

  @AutoLogOutput(key = "Subsystems/Shooter/getVelocity")
  public double getVelocity() {
    return (getSpeed() - ShooterConstants.rpmThreshold.get())
        / ShooterConstants.kShooterVelocityFactor.get();
  }

  public void runCharacterization(double output) {
    super.runCharacterization(output);
    follower.runCharacterization(-output);
  }

  @Override
  public void runFFVelocity(double speed) {
    super.runFFVelocity(speed);
    follower.runFFVelocity(-speed);
  }

  @Override
  public void stop() {
    super.stop();
    follower.stop();
  }
}
