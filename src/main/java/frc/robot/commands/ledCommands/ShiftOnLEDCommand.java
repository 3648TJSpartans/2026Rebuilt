// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.shiftTracker.ShiftTracker;

public class ShiftOnLEDCommand extends Command {
  private final LedSubsystem m_leds;
  private final ShiftTracker m_shiftTracker;
  private LEDPattern m_pattern;

  public ShiftOnLEDCommand(LedSubsystem leds, ShiftTracker shiftTracker, LEDPattern pattern) {
    m_leds = leds;
    m_shiftTracker = shiftTracker;
    addRequirements(m_leds, m_shiftTracker);
    m_pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leds.setPattern(
        m_leds.leftBuffer,
        m_pattern.mask(LEDPattern.progressMaskLayer(() -> m_shiftTracker.timeLeft() / 25)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leds.turnLedsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
