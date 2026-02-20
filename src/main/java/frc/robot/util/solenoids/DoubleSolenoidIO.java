package frc.robot.util.solenoids;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.Status;

public class DoubleSolenoidIO extends SolenoidIO {

  private DoubleSolenoid m_solenoid;

  public DoubleSolenoidIO(int solenoidChannel1, int solenoidChannel2, String name) {
    super(name);
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidChannel1, solenoidChannel2);
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

  // Returns WARNING because getStatus() has not been configured (this method should be overwritten)
  public Status getStatus() {
    return Status.WARNING;
  }

  @Override
  public void setSolenoid(boolean on) {
    // TODO Auto-generated method stub
    if (on) {
      setSolenoidForward();
      return;
    }
    setSolenoidReverse();
  }

  @Override
  public boolean getSolenoidOn() {
    return getForwardSolenoidEnabled();
  }

  @Override
  public boolean getSolenoidEnabled() {
    return getForwardSolenoidEnabled() || getReverseSolenoidEnabled();
  }
}
