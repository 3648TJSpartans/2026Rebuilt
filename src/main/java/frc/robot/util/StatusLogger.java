package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class StatusLogger extends SubsystemBase {

  Statusable[] m_statuses;

  public StatusLogger(Statusable... statuses) {
    m_statuses = statuses;
  }

  public void logStatuses() {
    for (Statusable status : m_statuses) {
      Logger.recordOutput("Utils/Statusable/" + status.getName(), status.getStatus());
    }
  }

  public Statusable[] getStatuses() {
    return m_statuses;
  }
}
