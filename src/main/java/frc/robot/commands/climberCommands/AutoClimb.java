package frc.robot.commands.climberCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import java.util.function.Supplier;

public class AutoClimb extends Command {
  private final Climber m_climber;
  private final Supplier<Double> m_angleSupplier;
  private PIDController m_pidController;
  private double m_maxPower;
  private double m_minPower;
  private double m_tolerance;
  private double m_goal;

  public AutoClimb(Climber climber, Supplier<Double> angleSupplier) {
    m_climber = climber;
    m_angleSupplier = angleSupplier;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_pidController = ClimberConstants.getPidController();
    m_maxPower = ClimberConstants.tunableAutoMaxPower.get();
    m_minPower = ClimberConstants.tunableAutoMinPower.get();
    m_tolerance = ClimberConstants.tunablAutoTolerance.get();
    m_goal = ClimberConstants.tunableGoal().get();
  }

  @Override
  public void execute() {
    double angle = m_angleSupplier.get();
    double offset = 
    double power = 
  }

  @Override
  public void end(boolean interupted) {
    m_climber.stop();
  }
}
