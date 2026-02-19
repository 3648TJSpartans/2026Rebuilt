package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorUtil.SparkIO;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements Statusable {
  private final SparkIO leader;
  private final SparkIO follower;

  public Shooter(SparkIO leader, SparkIO follower) {
    this.leader = leader;
    this.follower = follower;
  }

  public void setPower(double power) {
    leader.setPower(power);
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
    return (leader.getSpeed() - ShooterConstants.rpmThreshold.get())
        / ShooterConstants.kShooterVelocityFactor.get();
  }

  public void runCharacterization(double output) {
    leader.runCharacterization(output);
    follower.runCharacterization(-output);
  }

  public void runFFVelocity(double speed) {
    leader.runFFVelocity(speed);
    follower.runFFVelocity(-speed);
  }

  public void stop() {
    leader.stop();
    follower.stop();
  }

  public SparkIO getLeaderMotor() {
    return leader;
  }
}
