package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.vision.Neural;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RangeCalc;
import java.util.function.Supplier;

public class LedPeriodicCommand extends Command {
  private final LedSubsystem m_leds;
  private final Drive m_drive;
  private final Neural m_neural;
  private final Vision m_vision;
  private final Supplier<Boolean> m_override;

  public LedPeriodicCommand(
      LedSubsystem leds, Drive drive, Neural neural, Vision vision, Supplier<Boolean> override) {
    m_leds = leds;
    m_drive = drive;
    m_override = override;
    m_neural = neural;
    m_vision = vision;

    addRequirements(m_leds);
  }

  @Override
  public void execute() { // Shooter status
    if (RangeCalc.inShootingRange(m_drive.getPose())) {
      m_leds.setPattern(m_leds.centerBuffer1, LedConstants.green);
    } else {
      m_leds.setPattern(m_leds.centerBuffer1, LedConstants.red);
    }

    if (m_vision.getPipeline(0)
        == 0) { // Vision status - blue if detecting AprilTags, yellow/purple if in neural mode and
      // detecting/not detecting balls
      m_leds.setPattern(m_leds.centerBuffer2, LedConstants.blue);
    } else if (m_neural.isPoseDetected()) {
      m_leds.setPattern(m_leds.centerBuffer2, LedConstants.yellow);
    } else {
      m_leds.setPattern(m_leds.centerBuffer2, LedConstants.purple);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
