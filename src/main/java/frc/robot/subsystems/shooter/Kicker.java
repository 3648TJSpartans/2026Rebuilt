package frc.robot.subsystems.shooter;

import frc.robot.Constants.Status;
import frc.robot.util.StatusableDigitalInput;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Kicker extends RelEncoderSparkMax {

  private final StatusableDigitalInput irSensor;

  public Kicker() {
    super(ShooterConstants.kKickerMotorConfig);
    irSensor =
        new StatusableDigitalInput(
            ShooterConstants.kickerIRSensorChannel, "Subsystems/Kicker/irSensor");
  }

  public boolean getSensor() {
    return !irSensor.get();
  }

  public void runExceptSensor(double speed) {
    if (!irSensor.get()) {
      setSpeed(speed);
    }
  }

  @Override
  public void updateValues() {
    super.updateValues();
    Logger.recordOutput("Subsystems/Kicker/irSensor", irSensor.get());
  }

  @Override
  public String getName() {
    return "Subsystems/Kicker";
  }

  @Override
  public Status getStatus() {
    if (super.getStatus() != Status.OK) {
      Logger.recordOutput("Debug/Subsystems/Kicker/error", "Motor not attatched");
      return super.getStatus();
    }
    if (irSensor.getStatus() != Status.OK) {
      Logger.recordOutput("Debug/Subsystems/Kicker/warning", "IR Sensor");
      return irSensor.getStatus();
    }
    return Status.OK;
  }
}
