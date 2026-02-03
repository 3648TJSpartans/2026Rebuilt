package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DoubleSolenoidIO extends SubsystemBase {

  private DoubleSolenoid m_solenoid;
  private String name;

  public DoubleSolenoidIO(int solenoidChannel1, int solenoidChannel2, String name) {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidChannel1, solenoidChannel2);
    this.name = name;
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
}
