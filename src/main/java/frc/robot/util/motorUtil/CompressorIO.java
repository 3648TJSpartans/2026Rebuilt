package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CompressorIO extends SubsystemBase {

  private Compressor m_compressor;

  public CompressorIO() {
    m_compressor = new Compressor(PneumaticsModuleType.REVPH);
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

  public boolean getCompressorFull() {
    return m_compressor.getPressureSwitchValue();
  }

  public boolean getCompressorEnabled() {
    return m_compressor.isEnabled();
  }

  public void updateValues() {
    Logger.recordOutput("Subsystems/Compressor/getCompressorEnabled", getCompressorEnabled());
    Logger.recordOutput("Subsystems/Compressor/getCompressorFull", getCompressorFull());
    Logger.recordOutput("Subsystems/Compressor/getCompressorCurrent", m_compressor.getCurrent());
    Logger.recordOutput("Subsystems/Compressor/getCompressorPressure", m_compressor.getPressure());
    Logger.recordOutput("Subsystems/Compressor/getCompressorAnalogVoltage", m_compressor.getAnalogVoltage());
  }

  @Override
  public void periodic() {
    updateValues();
  }
}
