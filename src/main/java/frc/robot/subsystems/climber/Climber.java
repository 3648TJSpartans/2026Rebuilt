package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Climber extends RelEncoderSparkMax {
  private final DigitalOutput bottomSwitch;
  private final DigitalOutput topSwitch;
  private final RelEncoderSparkMax follower;

  public Climber() {
    super(ClimberConstants.leadMotorConfig);
    bottomSwitch = new DigitalOutput(ClimberConstants.bottomSwitchPort);
    topSwitch = new DigitalOutput(ClimberConstants.topSwitchPort);
    follower = new RelEncoderSparkMax(ClimberConstants.followMotorConfig);
  }

  @Override
  public void setPosition(double position) {
    position =
        MathUtil.clamp(
            position, ClimberConstants.minPosition.get(), ClimberConstants.maxPosition.get());
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

  public Rotation3d getRotation() {
    return new Rotation3d(getPosition() * ClimberConstants.encoderPositionFactor, 0.0, 0.0);
  }

  public void setRotation(double rotation) {
    setPosition(rotation / ClimberConstants.encoderPositionFactor);
  }

  @Override
  public void periodic() {
    super.periodic();
    updateInputs();
  }

  public void updateInputs() {
    Logger.recordOutput("Subsystems/Climber/position", getRotation());
  }
}
