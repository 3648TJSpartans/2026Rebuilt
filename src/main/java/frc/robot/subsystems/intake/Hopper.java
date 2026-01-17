package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  public Hopper() {}

  public void setHopperSpeed(double speed) {
    IntakeConstants.hopperMotor.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    IntakeConstants.indexerMotor.set(speed);
  }

  public void setHopperAndIndexerSpeed(double hopperSpeed, double indexerSpeed) {
    IntakeConstants.hopperMotor.set(indexerSpeed);
    IntakeConstants.indexerMotor.set(hopperSpeed);
  }

  public void stopHopperAndIndexer() {
    IntakeConstants.hopperMotor.set(0);
    IntakeConstants.indexerMotor.set(0);
  }

  public double getHopperSpeed() {
    return IntakeConstants.hopperMotor.get();
  }

  public double getIndexerSpeed() {
    return IntakeConstants.indexerMotor.get();
  }

  public void updateValues() {
    Logger.recordOutput(IntakeConstants.motorConfig.name() + "/hopperSpeed", getHopperSpeed());
    Logger.recordOutput(IntakeConstants.motorConfig.name() + "/indexerSpeed", getIndexerSpeed());
  }

  @Override
  public void periodic() {
    updateValues();
  }
}
