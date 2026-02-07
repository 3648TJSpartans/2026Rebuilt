package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public class StatusCheckLEDCommand extends Command {
  private final Statusable[] m_statuses;
  private final LedSubsystem m_leds;

  public StatusCheckLEDCommand(LedSubsystem leds, Statusable... statuses) {
    m_statuses = statuses;
    m_leds = leds;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", true);
  }

  @Override
  public void execute() {
    for (int i = 0; i < m_statuses.length; i++) {
      switch (m_statuses[i].getStatus()) {
        case OK -> {
          m_leds.setSingleLed(0, 255, 0, i);
          Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "OK");
        }
        case WARNING -> {
          m_leds.setSingleLed(255, 0, 255, i);
          Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "WARNING");
        }
        case UNKNOWN -> {
          m_leds.setSingleLed(255, 0, 175, i);
          Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "UNKNOWN");
        }
        case ERROR -> {
          m_leds.setSingleLed(255, 0, 0, i);
          Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "ERROR");
        }
      }
    }
  }

  @Override
  public void end(boolean interupted) {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", false);
  }
}
