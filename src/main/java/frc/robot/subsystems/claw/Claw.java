package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Claw extends RelEncoderSparkMax {
  private final DigitalOutput bottomSwitch;
  private final DigitalOutput topSwitch;

  public Claw() {
    super(ClawConstants.motorConfig);
    bottomSwitch = new DigitalOutput(ClawConstants.bottomSwitchPort);
    topSwitch = new DigitalOutput(ClawConstants.topSwitchPort);
  }

  @Override
  public void setPosition(double position) {
    position =
        MathUtil.clamp(position, ClawConstants.minPosition.get(), ClawConstants.maxPosition.get());
    if (topSwitch.get()) {
      if (position > getPosition()) {
        super.setPower(0.0);
        return;
      }
    }

    if (bottomSwitch.get()) {
      if (position < getPosition()) {
        super.setPower(0.0);
        return;
      }
    }
    super.setPosition(position);
  }

  public Translation3d getTranslation() {
    return new Translation3d(0, 0, getPosition() * ClawConstants.encoderPositionFactor);
  }

  public void setHeight(double height) {
    setPosition(height / ClawConstants.encoderPositionFactor);
  }

  @Override
  public void periodic() {
    super.periodic();
    updateInputs();
  }

  public void updateInputs() {
    Logger.recordOutput("Subsystems/Claw/position", getTranslation());
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }

  public void setPower(double power) {
    super.setPower(power);
  }

  @Override
  public void stop() {
    super.setPower(0.0);
  }
}
