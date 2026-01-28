package frc.robot.commands.climberCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
    m_goal = ClimberConstants.tunableGoal.get();

    Logger.recordOutput("Commands/Climber/AutoClimb/maxPower", m_maxPower);
    Logger.recordOutput("Commands/Climber/AutoClimb/minPower", m_minPower);
    Logger.recordOutput("Commands/Climber/AutoClimb/tolerance", m_tolerance);
    Logger.recordOutput("Commands/Climber/AutoClimb/goal", m_goal);
  }

  @Override
  public void execute() {
    double angle = m_angleSupplier.get();
    double offset =
        m_goal - angle; // TODO if it spins the wrong way when testign, reverse these values.
    double power = MathUtil.clamp(m_pidController.calculate(offset), m_minPower, m_maxPower);
    m_climber.setPower(power);
  }

  @Override
  public void end(boolean interupted) {
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_angleSupplier.get() - m_goal) < m_tolerance;
  }
}
