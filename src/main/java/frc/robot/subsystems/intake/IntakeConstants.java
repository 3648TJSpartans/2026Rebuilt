package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.motorUtil.MotorConfig;

public class IntakeConstants {

  public static final MotorConfig hopperMotorConfig =
      new MotorConfig("Subsystems/Intake/MotorIO")
          .motorCan(15)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final SparkMax intakeRollerMotor = new SparkMax(16, MotorType.kBrushless);

  public static final double intakeRollerSpeed = 0.01;
  public static final double hopperSpeed = 0.01;
  public static final double indexerSpeed = 0.01;

  public static final double upPosition = 0;
  public static final double downPosition = 0;

  public static final int solenoidChannel = 1;
}
