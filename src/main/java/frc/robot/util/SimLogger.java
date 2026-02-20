package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class SimLogger extends SubsystemBase {
  private final Turret m_turret;
  private final Intake m_intake;

  public SimLogger(Turret turret, Intake intake) {
    m_turret = turret;
    m_intake = intake;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Utils/SimLogger/componentPositions",
        new Pose3d[] {
          new Pose3d(TurretConstants.kTurretOffset, new Rotation3d(m_turret.getTurretRotation())),
          new Pose3d(
              IntakeConstants.intakeOffset,
              m_intake.getSolenoid().getSolenoidOn()
                  ? IntakeConstants.intakeDownRotation
                  : IntakeConstants.intakeUpRotation)
        });
  }
}
