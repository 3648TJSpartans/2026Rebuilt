package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.SparkIO;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends SubsystemBase implements Statusable {
  private final SparkIO motor;

  public Hood(SparkIO motor) {
    this.motor = motor;
  }

  // public void setAngle(double rotation) {
  //   double setpoint = rotation * HoodConstants.hoodEncoderFactor;
  //   setPosition(setpoint);
  // }

  @Override
  public void periodic(){
    super.periodic();
    motor.periodic();
    if(motor.getPosition()>0.95){
      motor.setEncoder(0.0);
    }
  }

  @AutoLogOutput(key = "Subsystems/Hood/getAngle")
  public double getAngle() {
    if (!Constants.hoodWorking.get()) {
      return Units.degreesToRadians(HoodConstants.minAngle.get());
    }
    return motor.getPosition() / HoodConstants.hoodEncoderFactor.getAsDouble()
        + Units.degreesToRadians(HoodConstants.minAngle.get());
  }

  public void setPosition(double setpoint) {
    setpoint =
        MathUtil.clamp(setpoint, HoodConstants.minPosition.get(), HoodConstants.maxPosition.get());
    motor.setPosition(setpoint);
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
    return motor.getStatus();
  }

  public SparkIO getMotor() {
    return motor;
  }
}
