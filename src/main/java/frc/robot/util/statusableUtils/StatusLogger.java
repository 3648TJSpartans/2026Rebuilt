package frc.robot.util.statusableUtils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
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
    if (m_statuses == null) {
      return new Statusable[] {
        new Statusable() {
          @Override
          public Status getStatus() {
            return Status.UNKNOWN;
          }

          @Override
          public String getName() {
            return "Default Status";
          }
        }
      };
    }
    return m_statuses;
  }
}
