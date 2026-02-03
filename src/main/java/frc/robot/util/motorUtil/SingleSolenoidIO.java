package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SingleSolenoidIO extends SubsystemBase {

  private Solenoid m_solenoid;
  private String name;

  public SingleSolenoidIO(int channel, String name) {
    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, channel);
    this.name = name;
  }

  public void setSolenoid(boolean on) {
    m_solenoid.set(on);
  }

  public void toggleSolenoid() {
    m_solenoid.toggle();
  }

  public boolean getSolenoidOn() {
    return m_solenoid.get();
  }

  public boolean getSolenoidEnabled() {
    return !m_solenoid.isDisabled();
  }

  public void updateValues() {
    Logger.recordOutput(name + "/getSolenoidOn", getSolenoidOn());
    Logger.recordOutput(name + "/getSolenoidEnabled", getSolenoidEnabled());
  }

  @Override
  public void periodic() {
    updateValues();
  }
}
