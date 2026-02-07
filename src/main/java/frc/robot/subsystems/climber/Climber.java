package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Climber extends RelEncoderSparkMax {
  private final RelEncoderSparkMax follower;

  public Climber() {
    super(ClimberConstants.leadMotorConfig);
    follower = new RelEncoderSparkMax(ClimberConstants.followMotorConfig);
  }

  @Override
  public void setPosition(double position) {
    position =
        MathUtil.clamp(
            position, ClimberConstants.minPosition.get(), ClimberConstants.maxPosition.get());
    super.setPosition(position);
  }

  public Translation3d getTranslation() {
    return new Translation3d(0, 0, getPosition() * ClimberConstants.encoderPositionFactor);
  }

  public void setHeight(double height) {
    setPosition(height / ClimberConstants.encoderPositionFactor);
  }

  @Override
  public void periodic() {
    super.periodic();
    updateInputs();
  }

  public void updateInputs() {
    Logger.recordOutput("Subsystems/Climber/position", getTranslation());
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }

  public void setPower(double power) {
    super.setPower(power);
    follower.setPower(power);
  }

  @Override
  public void stop() {
    super.setPower(0.0);
    follower.setPower(0.0);
  }
}
