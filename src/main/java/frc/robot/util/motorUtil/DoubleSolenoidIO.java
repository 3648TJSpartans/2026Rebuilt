package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import frc.robot.util.Statusable;
import org.littletonrobotics.junction.Logger;

public class DoubleSolenoidIO extends SubsystemBase implements Statusable {

  private DoubleSolenoid m_solenoid;
  private String name;

  public DoubleSolenoidIO(int solenoidChannel1, int solenoidChannel2, String name) {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidChannel1, solenoidChannel2);
    this.name = name;
  }

  @Override
  public final String getName() {
    return name;
  }

  public void setSolenoidForward() {
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setSolenoidReverse() {
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setSolenoidOff() {
    m_solenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void toggleSolenoid() {
    m_solenoid.toggle();
  }

  public boolean getForwardSolenoidOn() {
    return m_solenoid.get() == Value.kForward;
  }

  public boolean getReverseSolenoidOn() {
    return m_solenoid.get() == Value.kReverse;
  }

  public boolean getForwardSolenoidEnabled() {
    return !m_solenoid.isFwdSolenoidDisabled();
  }

  public boolean getReverseSolenoidEnabled() {
    return !m_solenoid.isRevSolenoidDisabled();
  }

  public void updateValues() {
    Logger.recordOutput(name + "/getForwardSolenoidOn", getForwardSolenoidOn());
    Logger.recordOutput(name + "/getReverseSolenoidOn", getReverseSolenoidOn());
    Logger.recordOutput(name + "/getForwardSolenoidEnabled", getForwardSolenoidEnabled());
    Logger.recordOutput(name + "/getReverseSolenoidEnabled", getReverseSolenoidEnabled());
  }

  @Override
  public void periodic() {
    updateValues();
  }

  // Returns WARNING because getStatus() has not been configured (this method should be overwritten)
  public Status getStatus() {
    return Status.WARNING;
  }
}
