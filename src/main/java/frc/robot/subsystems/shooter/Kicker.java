package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Kicker extends RelEncoderSparkMax {

  private final DigitalOutput irSensor;

  public Kicker() {
    super(ShooterConstants.kKickerMotorConfig);
    irSensor = new DigitalOutput(ShooterConstants.kickerIRSensorChannel);
  }

  public boolean getSensor() {
    return irSensor.get();
  }

  public void runExceptSensor(double speed) {
    if (!irSensor.get()) {
      setSpeed(speed);
    }
  }

  @Override
  public void updateValues() {
    super.updateValues();
    Logger.recordOutput("Subsystems/Climber/irSensor", irSensor.get());
  }
}
