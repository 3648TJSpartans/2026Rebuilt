// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.relativeEncoderCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.relativeEncoder.RelEncoder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeRelCmd extends Command {
  // declare the motors and the substem
  private final RelEncoder m_relEncoder;

  /** Creates a new Elevator. */
  public HomeRelCmd(RelEncoder relEncoder) {
    m_relEncoder = relEncoder;
    addRequirements(relEncoder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("relEncoder/Command/Scheduled", "Homing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_relEncoder.setSpeed(-.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("relEncoder/Command/Scheduled", "Unscheduled");
    m_relEncoder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_relEncoder.atZero();
  }
}
