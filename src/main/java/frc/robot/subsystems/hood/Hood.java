package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Status;
import frc.robot.util.Statusable;
import frc.robot.util.motorUtil.AbsEncoderSparkMax;

public class Hood extends AbsEncoderSparkMax implements Statusable {
  public Hood() {
    super(HoodConstants.motorConfig);
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
    return Status.OK;
  }
}
