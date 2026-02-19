package frc.robot.subsystems.intake;

import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import frc.robot.util.solenoids.SolenoidIO;
import frc.robot.util.statusableUtils.Statusable;

public class Intake implements Statusable {

  private final RelEncoderSparkMax roller;
  private final SolenoidIO m_solenoid;

  public Intake(SolenoidIO solenoid) {
    m_solenoid = solenoid;
    roller = new RelEncoderSparkMax(IntakeConstants.intakeRollerConfig);
  }

  public void setRollers(double speed) {
    roller.setPower(speed);
  }

  public void stopRollers() {
    roller.stop();
  }

  public void setSolenoidAndRollerUp() {
    m_solenoid.setSolenoid(false);
    roller.stop();
  }

  public void setSolenoidAndRollerDown() {
    m_solenoid.setSolenoid(true);
    roller.setPower(IntakeConstants.intakeRollerSpeed.get());
  }

  public SolenoidIO getSolenoid() {
    return m_solenoid;
  }

  @Override
  public Status getStatus() {
    return m_solenoid.getStatus();
  }

  @Override
  public String getName() {
    return "Subsystems/Intake";
  }
}
