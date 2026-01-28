package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.RelEncoderSparkMax;
import frc.robot.util.motorUtil.SingleSolenoidIO;

public class Intake extends SingleSolenoidIO {

  private RelEncoderSparkMax roller;

  public Intake() {
    super(IntakeConstants.solenoidChannel, "Intake");
    roller = new RelEncoderSparkMax(IntakeConstants.intakeRollerConfig);
  }

  public void setRollers(double speed) {
    roller.setPower(speed);
  }

  public void setSolenoidAndRollerUp() {
    setSolenoid(false);
    roller.stop();
  }

  public void setSolenoidAndRollerDown() {
    setSolenoid(true);
    roller.setPower(IntakeConstants.intakeRollerSpeed.get());
  }
}
