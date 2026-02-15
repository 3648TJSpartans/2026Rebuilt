package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    return getPosition() / HoodConstants.hoodEncoderFactor.getAsDouble()
        + Units.degreesToRadians(HoodConstants.minAngle.get());
  }

  @Override
  public void setPosition(double setpoint) {
    setpoint =
        MathUtil.clamp(setpoint, HoodConstants.minPosition.get(), HoodConstants.maxPosition.get());
    super.setPosition(setpoint);
  }

  public void setAngle(Rotation2d angle) {
    double setpoint =
        angle
                .minus(new Rotation2d(Units.degreesToRadians(HoodConstants.minAngle.get())))
                .getRadians()
            * HoodConstants.hoodEncoderFactor.getAsDouble();
    setPosition(setpoint);
  }

  @Override
  public Status getStatus() {
    return super.getStatus();
  }
}
