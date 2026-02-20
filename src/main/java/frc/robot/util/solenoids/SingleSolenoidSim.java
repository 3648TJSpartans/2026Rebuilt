package frc.robot.util.solenoids;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.robot.Constants.Status;

public class SingleSolenoidSim extends SolenoidIO {

  private SolenoidSim m_solenoid;
  private boolean on;

  public SingleSolenoidSim(int channel, String name) {
    super(name);
    m_solenoid = new SolenoidSim(PneumaticsModuleType.REVPH, channel);
    on = false;
  }

  public void setSolenoid(boolean on) {
    this.on = on;
  }

  public void toggleSolenoid() {
    on = !on;
  }

  public boolean getSolenoidOn() {
    return on;
  }

  public boolean getSolenoidEnabled() {
    return true;
  }

  public Status getStatus() {
    return Status.UNKNOWN;
  }
}
