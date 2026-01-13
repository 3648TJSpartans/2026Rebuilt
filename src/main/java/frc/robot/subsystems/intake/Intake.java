package frc.robot.subsystems.intake;

import frc.robot.util.motorUtil.MultiMotorSubsystem;

public class Intake extends MultiMotorSubsystem {

  public Intake() {
    super(IntakeConstants.motor0, IntakeConstants.motor1, IntakeConstants.motor2);
  }
}
