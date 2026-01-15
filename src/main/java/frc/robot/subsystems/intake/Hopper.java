package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  public Hopper() {}

  public void setHopperSpeed(double speed) {
    IntakeConstants.hopperMotor.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    IntakeConstants.indexerMotor.set(speed);
  }

  public void setHopperAndIndexSpeed(double hopperSpeed, double indexerSpeed) {
    IntakeConstants.hopperMotor.set(indexerSpeed);

    IntakeConstants.indexerMotor.set(hopperSpeed);
  }
}
