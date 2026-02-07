package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.AbsEncoderSparkMax;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends AbsEncoderSparkMax {
  public Hood() {
    super(HoodConstants.motorConfig);
  }

  public void setAngle(double rotation) {
    double setpoint = rotation * HoodConstants.hoodEncoderFactor.get();
    setPosition(setpoint);
  }

  @AutoLogOutput
  public double getAngle() {
    return getPosition() / HoodConstants.hoodEncoderFactor.get();
  }

  @Override
  public void setPosition(double setpoint) {
    setpoint = MathUtil.clamp(setpoint, HoodConstants.minAngle.get(), HoodConstants.maxAngle.get());
    super.setPosition(setpoint);
  }

  public void setAngle(Rotation2d angle) {
    setPosition(angle.getRadians() * HoodConstants.hoodEncoderFactor.get());
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }
}
