package frc.robot.subsystems.vortexMotor;

import frc.robot.util.motorUtil.MotorConfig;

public class VortexMotorConstants {

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/VortexMotor/MotorIO")
          .motorCan(20)
          .p(0.0)
          .i(0.0)
          .d(0.0)
          .maxPower(0.2)
          .minPower(-0.2)
          .positionTolerance(0.01)
          .speedTolerance(0.01);
}
