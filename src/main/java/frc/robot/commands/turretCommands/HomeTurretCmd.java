package frc.robot.commands.turretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class HomeTurretCmd extends Command {
  private final Turret m_turret;
  private final double power;
  private final double range;

  public HomeTurretCmd(Turret turret) {
    m_turret = turret;
    power = TurretConstants.homePower;
    range = TurretConstants.homeRange;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/HomeTurret/power", power);
    Logger.recordOutput("Commands/HomeTurret/range", range);
    m_turret.setPower(power);
  }

  @Override
  public void execute() {
    double turretRot = m_turret.getTurretRotation().getRadians();
    Logger.recordOutput("Commands/HomeTurret/turretRotation", turretRot);
    if (turretRot > range) {
      m_turret.setPower(-power);
      return;
    }
    if (turretRot < -range) {
      m_turret.setPower(power);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  @Override
  public boolean isFinished() {
    return m_turret.isHomed();
  }
}
