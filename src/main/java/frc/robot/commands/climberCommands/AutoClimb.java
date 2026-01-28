package frc.robot.commands.climberCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.util.LoggedCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoClimb extends LoggedCommand {
  private final Climber m_climber;
  private final Supplier<Double> m_angleSupplier;
  private PIDController m_pidController;
  private double m_maxPower;
  private double m_minPower;
  private double m_tolerance;
  private double m_goal;

  public AutoClimb(Climber climber, Supplier<Double> angleSupplier) {
    super("Commands/Climber/AutoClimb/outputs");
    m_climber = climber;
    m_angleSupplier = angleSupplier;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_pidController = ClimberConstants.getPidController();
    m_maxPower = ClimberConstants.tunableAutoMaxPower.get();
    m_minPower = ClimberConstants.tunableAutoMinPower.get();
    m_tolerance = ClimberConstants.tunablAutoTolerance.get();
    m_goal = ClimberConstants.tunableGoal.get();

    Logger.recordOutput("Commands/Climber/AutoClimb/inputs/maxPower", m_maxPower);
    Logger.recordOutput("Commands/Climber/AutoClimb/inputs/minPower", m_minPower);
    Logger.recordOutput("Commands/Climber/AutoClimb/inputs/tolerance", m_tolerance);
    Logger.recordOutput("Commands/Climber/AutoClimb/inputs/goal", m_goal);
  }

  @Override
  public void execute() {
    double angle = m_angleSupplier.get();
    Logger.recordOutput("Commands/Climber/AutoClimb/outputs/angle", angle);
    double offset =
        m_goal - angle; // TODO if it spins the wrong way when testign, reverse these values.
    Logger.recordOutput("Commands/Climber/AutoClimb/outputs/offset", offset);
    double power = MathUtil.clamp(m_pidController.calculate(offset), m_minPower, m_maxPower);
    Logger.recordOutput("Commands/Climber/AutoClimb/outputs/power", power);
    m_climber.setPower(power);
  }

  @Override
  public void end(boolean interupted) {
    super.end(interupted);
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_angleSupplier.get() - m_goal) < m_tolerance;
  }
}
