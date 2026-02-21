package frc.robot.commands.climberCommands;

import frc.robot.subsystems.theClaw.TheClaw;
import frc.robot.subsystems.theClaw.TheClawstants;
import frc.robot.util.LoggedCommand;
import org.littletonrobotics.junction.Logger;

public class HomeClimber extends LoggedCommand {

  private final TheClaw m_claw;
  private double power;

  public HomeClimber(TheClaw claw) {
    super("Commands/Climber/HomeClimber/outputs");
    m_claw = claw;
    addRequirements(m_claw);
  }

  @Override
  public void initialize() {
    super.initialize();
    power = TheClawstants.homePower.get();
    Logger.recordOutput("Commands/Climber/HomeClimber/homePower", power);
  }

  @Override
  public void execute() {
    m_claw.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_claw.stop();
  }

  @Override
  public boolean isFinished() {
    return m_claw.isHomed();
  }
}
