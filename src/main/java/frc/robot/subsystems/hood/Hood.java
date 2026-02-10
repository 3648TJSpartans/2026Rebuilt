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

  // public void setAngle(double rotation) {
  //   double setpoint = rotation * HoodConstants.hoodEncoderFactor;
  //   setPosition(setpoint);
  // }

  @AutoLogOutput(key = "Subsystems/Hood/getAngle")
  public double getAngle() {
    return getPosition() / HoodConstants.hoodEncoderFactor + HoodConstants.minAngle.getRadians();
  }

  @Override
  public void setPosition(double setpoint) {
    setpoint =
        MathUtil.clamp(setpoint, HoodConstants.minPosition.get(), HoodConstants.maxPosition.get());
    super.setPosition(setpoint);
  }

  public void setAngle(Rotation2d angle) {
    double setpoint =
        (MathUtil.clamp(
                    angle.getRadians(),
                    HoodConstants.minAngle.getRadians(),
                    HoodConstants.maxAngle.getRadians())
                - HoodConstants.minAngle.getRadians())
            * HoodConstants.hoodEncoderFactor;
    setPosition(setpoint);
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }
}
