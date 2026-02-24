package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import frc.robot.util.Supplier;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import frc.robot.util.solenoids.SolenoidIO;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Statusable {

  private final RelEncoderSparkMax roller;
  private final SolenoidIO m_solenoid;
  public IntakeState m_state;

  public static enum IntakeState {
    UP,
    DOWN
  }

  public Intake(SolenoidIO solenoid, Supplier<Pose2d> drivePose) {
    m_solenoid = solenoid;
    roller = new RelEncoderSparkMax(IntakeConstants.intakeRollerConfig);
  }

  public IntakeState getIntakeState() {
    return m_state;
  }

  public void setRollers(double speed) {
    roller.setPower(speed);
  }

  public void stopRollers() {
    roller.stop();
  }

  public void setSolenoidAndRollerUp() {
    m_solenoid.setSolenoid(false);
    roller.stop();
  }

  public void setSolenoidAndRollerDown() {
    m_solenoid.setSolenoid(true);
    roller.setPower(IntakeConstants.intakeRollerSpeed.get());
  }

  public SolenoidIO getSolenoid() {
    return m_solenoid;
  }

  @Override
  public void periodic() {
    m_state = getSolenoid().getSolenoidOn() ? IntakeState.DOWN : IntakeState.UP;
    Logger.recordOutput("Subsystems/Intake/state", m_state.toString());
  }

  @Override
  public Status getStatus() {
    return m_solenoid.getStatus();
  }

  @Override
  public String getName() {
    return "Subsystems/Intake";
  }
}
