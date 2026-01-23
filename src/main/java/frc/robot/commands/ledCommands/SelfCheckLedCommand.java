package frc.robot.commands.ledCommands;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.opencv.objdetect.GraphicalCodeDetector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.turret.Turret;

public class SelfCheckLedCommand extends Command {
  private final LedSubsystem m_leds;
  private final List<SubsystemBase> m_listOfSubsystems;

  public SelfCheckLedCommand(LedSubsystem leds, List<SubsystemBase> listOfSubsystems) {
    super();
    m_leds = leds;
    m_listOfSubsystems = listOfSubsystems;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/LEDs/SelfCheckLedCommandInitialize", true);

  }

  @Override
  public void execute() {
    // 1 Give me a for loop
    // 1.5 Give me a for loop that is the length of the m_list_of_subsystems hint...
    // it's 2
    // 2 As part of that loop. Give me the index
    // 3 As part of the loop. Give me the subsystem that is at that index
    //
    Logger.recordOutput("Commands/LEDs/SelfCheckLedCommandExecute", true);

    for (int i = 0; i < m_listOfSubsystems.size(); i++) {
      // m_list_of_subsystems.size();
      SubsystemBase subsystem = (Turret) m_listOfSubsystems.get(i);
      try {
        m_leds.setSingleLedByStatus(i, subsystem.getStatus());
      } catch (Exception e) {
        // TODO: handle exception
      }
    }

    // m_leds.setSingleLed(0, 255, 0, 1);
    // m_leds.setSingleLed(255, 175, 0, 2);
    // m_leds.setSingleLed(255, 0, 0, 3);
  }

}
