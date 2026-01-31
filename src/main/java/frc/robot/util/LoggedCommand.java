package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class LoggedCommand extends Command {
  private final String m_name;

  public LoggedCommand(String name) {
    m_name = name;
  }

  @Override
  public void initialize() {
    Logger.recordOutput(m_name + "/isRunning", true);
  }

  @Override
  public void end(boolean interupted) {
    Logger.recordOutput(m_name + "/isRunning", false);
  }
}
