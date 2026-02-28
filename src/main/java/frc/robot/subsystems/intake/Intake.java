package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Status;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
import frc.robot.util.solenoids.SolenoidIO;
import frc.robot.util.statusableUtils.Statusable;
import frc.robot.util.zoneCalc.Polygon;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Statusable {

  private final RelEncoderSparkMax roller;
  private final SolenoidIO m_upSolenoid;
  private final SolenoidIO m_downSolenoid;
  public IntakeState m_state;
  private final Supplier<Pose2d> m_poseSupplier;
  private Polygon m_polygon;

  public static enum IntakeState {
    UP,
    DOWN
  }

  public Intake(SolenoidIO upSolenoid, SolenoidIO downSolenoid, Supplier<Pose2d> drivePose) {
    m_upSolenoid = upSolenoid;
    m_downSolenoid = downSolenoid;
    roller = new RelEncoderSparkMax(IntakeConstants.intakeRollerConfig);
    m_poseSupplier = drivePose;
    updatePolygon();
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
    m_upSolenoid.setSolenoid(true);
    m_downSolenoid.setSolenoid(false);
    roller.stop();
  }

  public void setSolenoidAndRollerDown() {
    m_upSolenoid.setSolenoid(false);
    m_downSolenoid.setSolenoid(true);
    roller.setPower(IntakeConstants.intakeRollerSpeed.get());
  }

  public SolenoidIO getUpSolenoid() {
    return m_upSolenoid;
  }

  public SolenoidIO getDownSolenoid() {
    return m_downSolenoid;
  }

  @Override
  public void periodic() {
    m_state = getDownSolenoid().getSolenoidOn() ? IntakeState.DOWN : IntakeState.UP;
    Logger.recordOutput("Subsystems/Intake/state", m_state.toString());
    updatePolygon();
  }

  @Override
  public Status getStatus() {
    return Constants.leastCommonStatus(m_upSolenoid.getStatus(), m_downSolenoid.getStatus());
  }

  @Override
  public String getName() {
    return "Subsystems/Intake";
  }

  public Polygon getPolygon() {
    return m_polygon;
  }

  private void updatePolygon() {
    m_polygon = getPolygon(m_poseSupplier.get(), m_state);
  }

  public Polygon getPolygon(Pose2d pose, IntakeState state) {
    Translation2d[] corners =
        state == IntakeState.UP ? IntakeConstants.cornersUp : IntakeConstants.cornersDown;
    return new Polygon(
        "Subsystems/Intake",
        Arrays.stream(corners)
            .map(corner -> corner.rotateBy(pose.getRotation()).plus(pose.getTranslation()))
            .toArray(Translation2d[]::new));
  }
}
