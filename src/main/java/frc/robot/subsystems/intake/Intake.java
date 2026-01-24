package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.SingleSolenoidIO;
import org.littletonrobotics.junction.Logger;

public class Intake extends SingleSolenoidIO {

  public Intake() {
    super(IntakeConstants.solenoidChannel, "Intake");
  }

  public void setRollerSpeed(double speed) {
    IntakeConstants.intakeRollerMotor.set(speed);
  }

  public void stopRoller() {
    IntakeConstants.intakeRollerMotor.stopMotor();
  }

  public void setSolenoidAndRollerUp() {
    setSolenoid(false);
    stopRoller();
  }

  public void setSolenoidAndRollerDown() {
    setSolenoid(true);
    setRollerSpeed(IntakeConstants.intakeRollerSpeed.get());
  }

  public double getRollerSpeed() {
    return IntakeConstants.intakeRollerMotor.get();
  }

  @Override
  public void updateValues() {
    super.updateValues();
    Logger.recordOutput("Intake" + "/intakeRollerSpeed", getRollerSpeed());
  }
}
