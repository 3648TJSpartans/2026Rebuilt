// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.*;

public class AutoLEDCommand extends Command {
  private final LedSubsystem m_leds;

  public AutoLEDCommand(LedSubsystem leds) {
    m_leds = leds;
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leds.setGlobalPattern(LedConstants.breathingGreen);
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
