package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.util.Statusable;
import org.littletonrobotics.junction.Logger;

public class StatusCheckLEDCommand extends Command {
  private final Statusable[] m_statuses;
  private final LedSubsystem m_leds;

  public StatusCheckLEDCommand(LedSubsystem leds, Statusable... statuses) {
    m_statuses = statuses;
    m_leds = leds;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", true);
  }

  @Override
  public void end(boolean interupted) {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", false);
  }
}
