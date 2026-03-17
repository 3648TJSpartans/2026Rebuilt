package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.util.shiftTracker.ShiftTracker;
import frc.robot.util.statusableUtils.Statusable;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class StatusCheckLEDCommand extends Command {
  private final Statusable[] m_statuses;
  private final LedSubsystem m_leds;
  private final ShiftTracker m_shiftTracker;
  private final BooleanSupplier shooting;

  public StatusCheckLEDCommand(
      LedSubsystem leds,
      ShiftTracker shiftTracker,
      BooleanSupplier shooting,
      Statusable... statuses) {
    m_statuses = statuses;
    m_leds = leds;
    m_shiftTracker = shiftTracker;
    this.shooting = shooting;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", true);
  }

  @Override
  public void execute() {
    if (DriverStation.isEnabled()) {
      if (m_shiftTracker.whichShift().equals("blue")) {
        if (shooting.getAsBoolean()) {
          m_leds.setGlobalPattern(LedConstants.blinkingBlue);
          return;
        }
        m_leds.setGlobalPattern(LedConstants.blue);
        return;
      }
      if (m_shiftTracker.whichShift().equals("red")) {
        if (shooting.getAsBoolean()) {
          m_leds.setGlobalPattern(LedConstants.red); // add blinking red
          return;
        }
        m_leds.setGlobalPattern(LedConstants.red);
        return;
      }
      if (shooting.getAsBoolean()) {
        m_leds.setGlobalPattern(LedConstants.red); // add blinking blue and red
        return;
      }
      m_leds.setGlobalPattern(LedConstants.red); // add blinking blue and red
      return;
    } else {
      for (int i = 0; i < m_statuses.length; i++) {
        if (m_statuses[i].getStatus() == null) {
          continue;
        }
        switch (m_statuses[i].getStatus()) {
          case OK -> {
            m_leds.setSingleLed(0, 255, 0, i + LedConstants.statusCheckOffset);
            Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "OK");
          }
          case WARNING -> {
            m_leds.setSingleLed(255, 0, 255, i + LedConstants.statusCheckOffset);
            Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "WARNING");
          }
          case UNKNOWN -> {
            m_leds.setSingleLed(255, 255, 255, i + LedConstants.statusCheckOffset);
            Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "UNKNOWN");
          }
          case ERROR -> {
            m_leds.setSingleLed(255, 0, 0, i + LedConstants.statusCheckOffset);
            Logger.recordOutput("Utils/Statusable/" + m_statuses[i].getName(), "ERROR");
          }
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/LED/StatusCheckLEDCommand/running", false);
  }
}
