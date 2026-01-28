package frc.robot.subsystems.examplePneumatic;

import frc.robot.util.motorUtil.DoubleSolenoidIO;
import frc.robot.util.motorUtil.SingleSolenoidIO;

public class Pneumatic extends DoubleSolenoidIO {

  public Pneumatic() {
    super(PneumaticConstants.solenoidChannel1, PneumaticConstants.solenoidChannel2, "examplePneumatic");
  }
}
