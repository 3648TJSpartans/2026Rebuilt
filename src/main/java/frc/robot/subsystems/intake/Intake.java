package frc.robot.subsystems.intake;

import frc.robot.RobotContainer.Status;
import frc.robot.util.motorUtil.AbsEncoderSparkMax;
import org.littletonrobotics.junction.Logger;

public class Intake extends AbsEncoderSparkMax {

  public Intake() {
    super(IntakeConstants.motorConfig);
  }

  public void setRollerSpeed(double speed) {
    IntakeConstants.intakeRollerMotor.set(speed);
  }

  public double getRollerSpeed() {
    return IntakeConstants.intakeRollerMotor.get();
  }

  @Override
  public void updateValues() {
    super.updateValues();
    Logger.recordOutput(
        IntakeConstants.motorConfig.name() + "/intakeRollerSpeed", getRollerSpeed());
  }

  public Status getStatus() {
    return Status.OK;
  }
}
