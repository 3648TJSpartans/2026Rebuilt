package frc.robot.util.solenoids;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public abstract class SolenoidIO extends SubsystemBase implements Statusable {
  private final String m_name;

  public SolenoidIO(String name) {
    m_name = name;
  }

  public final String getName() {
    return m_name;
  }

  public abstract void setSolenoid(boolean on);

  public abstract void toggleSolenoid();

  public abstract boolean getSolenoidOn();

  public abstract boolean getSolenoidEnabled();

  public void updateValues() {
    Logger.recordOutput(m_name + "/getSolenoidOn", getSolenoidOn());
    Logger.recordOutput(m_name + "/getSolenoidEnabled", getSolenoidEnabled());
  }

  @Override
  public void periodic() {
    updateValues();
  }

  public abstract Status getStatus();
}
