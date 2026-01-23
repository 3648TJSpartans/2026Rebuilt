package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotContainer.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Climber extends RelEncoderSparkMax {
  private final DigitalOutput bottomSwitch;
  private final DigitalOutput topSwitch;

  public Climber() {
    super(ClimberConstants.motorConfig);
    bottomSwitch = new DigitalOutput(ClimberConstants.bottomSwitchPort);
    topSwitch = new DigitalOutput(ClimberConstants.topSwitchPort);
  }

  @Override
  public void setPosition(double position) {
    position = MathUtil.clamp(
        position, ClimberConstants.minPosition.get(), ClimberConstants.maxPosition.get());
    if (topSwitch.get()) {
      if (position > getPosition()) {
        super.setSpeed(0.0);
        return;
      }
    }

    if (bottomSwitch.get()) {
      if (position < getPosition()) {
        super.setSpeed(0.0);
        return;
      }
    }
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

  public Status getStatus() {
    return Status.OK;
  }
}
