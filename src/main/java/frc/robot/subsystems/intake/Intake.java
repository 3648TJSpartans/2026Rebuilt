package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.AbsEncoderSparkMax;

public class Intake extends AbsEncoderSparkMax {

  public Intake() {
    super(IntakeConstants.motorConfig);
  }

  public void setRollerSpeed(double speed) {
    IntakeConstants.intakeRollerMotor.set(speed);
  }
}
