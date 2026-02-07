package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import frc.robot.util.motorUtil.SingleSolenoidIO;
import org.littletonrobotics.junction.Logger;

public class Climber extends RelEncoderSparkMax {
  private final DigitalOutput bottomSwitch;
  private final DigitalOutput topSwitch;
  private final RelEncoderSparkMax follower;
  private final SingleSolenoidIO solenoid =
      new SingleSolenoidIO(ClimberConstants.solenoidChannel, "Subsystems/Climber");

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
    Logger.recordOutput("Subsystems/Climber/powerToSpeed", getPowerToSpeed());
    Logger.recordOutput("Subsystems/Climber/currentToSpeed", getCurrentToSpeed());
    Logger.recordOutput("Subsystems/Climber/isEngaged", getEngaged());
    Logger.recordOutput("Subsystems/Climber/current", getCurrent());
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

  public double getPowerToSpeed() {
    return Math.abs(getSpeed()) > 100 && Math.abs(getAppliedOutput()) > .05
        ? Math.abs(getAppliedOutput() / getSpeed())
        : 0.0;
  }

  public double getCurrentToSpeed() {
    return Math.abs(getSpeed()) > 100 && Math.abs(getCurrent()) > .05
        ? Math.abs(getCurrent() / getSpeed())
        : 0.0;
  }

  public boolean getEngaged() {
    return getCurrentToSpeed() > ClimberConstants.currentToSpeedThreshold.get();
  }

  public void toggleSolenoid() {
    solenoid.toggleSolenoid();
  }
}
