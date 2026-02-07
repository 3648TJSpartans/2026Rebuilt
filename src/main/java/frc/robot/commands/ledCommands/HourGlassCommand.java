package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.HourGlass;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import java.util.function.DoubleSupplier;

public class HourGlassCommand extends Command {
  private final HourGlass m_hourGlass;
  private final DoubleSupplier m_tiltSupplier;
  private final LedSubsystem m_ledSubsystem;

  public HourGlassCommand(LedSubsystem ledSubsystem, DoubleSupplier tiltSupplier) {
    m_hourGlass = new HourGlass(10, 0.0);
    m_tiltSupplier = tiltSupplier;
    m_ledSubsystem = ledSubsystem;
  }

  @Override
  public void execute() {
    m_hourGlass.updateHourGlass(m_tiltSupplier.getAsDouble());
    m_ledSubsystem.setPattern(
        m_ledSubsystem.exampleView,
        LedConstants.white.mask(LEDPattern.progressMaskLayer(() -> m_hourGlass.getBottomVolume())));
  }
}
