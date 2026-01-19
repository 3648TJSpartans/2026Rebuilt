package frc.robot.subsystems.examplePneumatic;

import frc.robot.util.motorUtil.PneumaticIO;

public class Pneumatic extends PneumaticIO {

  public Pneumatic() {
    super(
        PneumaticConstants.solenoidChannel,
        PneumaticConstants.compressorChannel,
        "examplePneumatic");
  }
}
