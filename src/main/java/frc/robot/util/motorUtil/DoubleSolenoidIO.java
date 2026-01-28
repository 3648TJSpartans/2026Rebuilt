package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DoubleSolenoidIO extends SubsystemBase {

  private DoubleSolenoid m_solenoid;
  private Compressor m_compressor;
  private String name;

  public DoubleSolenoidIO(int solenoidChannel1, int solenoidChannel2, String name) {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidChannel1, solenoidChannel2);
    m_compressor = new Compressor(PneumaticsModuleType.REVPH);
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

  // Neither enable nor disable compressor should need to be used under normal conditions.
  // disableCompressor exists in case we want to add safety measures to our code.
  // It (and enableCompressor()) do not need to be used for any other reasons,
  // as the compressor automatically enables and disables when appropriate.
  public void disableCompressor() {
    m_compressor.disable();
  }

  public void enableCompressor() {
    m_compressor.enableDigital();
  }

  // public boolean getSolenoidOn() {
  //   return m_solenoid.get().toString();
  // }

  // public boolean getSolenoidEnabled() {
  //   return !m_solenoid.isDisabled();
  // }

  public boolean getCompressorFull() {
    return m_compressor.getPressureSwitchValue();
  }

  public boolean getCompressorEnabled() {
    return m_compressor.isEnabled();
  }

  public void updateValues() {
    // Logger.recordOutput(name + "/getSolenoidOn", getSolenoidOn());
    // Logger.recordOutput(name + "/getSolenoidDisabled", getSolenoidEnabled());
    Logger.recordOutput(name + "/getCompressorEnabled", getCompressorEnabled());
    Logger.recordOutput(name + "/getCompressorFull", getCompressorFull());
    Logger.recordOutput(name + "/getCompressorCurrent", m_compressor.getCurrent());
    Logger.recordOutput(name + "/getCompressorPressure", m_compressor.getPressure());
    Logger.recordOutput(name + "/getCompressorAnalogVoltage", m_compressor.getAnalogVoltage());
  }

  @Override
  public void periodic() {
    updateValues();
  }
}
