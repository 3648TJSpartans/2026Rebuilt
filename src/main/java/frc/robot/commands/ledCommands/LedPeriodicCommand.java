package frc.robot.commands.ledCommands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
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

    if (m_override.get()) {
      m_leds.setPattern(m_leds.centerBuffer2, LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5)));
    } else {
      m_leds.setPattern(m_leds.centerBuffer2, LedConstants.noColor);
    }

    if (m_vision.getPipeline(0)
        == 0) { // Vision status - blue if detecting AprilTags, yellow/purple if in neural mode and
      // detecting/not detecting balls
      m_leds.setPattern(m_leds.centerBuffer3, LedConstants.blue);
    } else if (m_neural.isPoseDetected()) {
      m_leds.setPattern(m_leds.centerBuffer3, LedConstants.yellow);
    } else {
      m_leds.setPattern(m_leds.centerBuffer3, LedConstants.purple);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
