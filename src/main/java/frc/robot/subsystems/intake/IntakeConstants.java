package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.motorUtil.MotorConfig;

public class IntakeConstants {

  public static final MotorConfig motorConfig =
      new MotorConfig("Subsystems/Intake/MotorIO")
          .motorCan(16)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final SparkMax intakeRollerMotor = new SparkMax(17, MotorType.kBrushless);
  public static final SparkMax hoppperRollerMotor = new SparkMax(18, MotorType.kBrushless);
  public static final SparkMax indexerRollerMotor = new SparkMax(19, MotorType.kBrushless);

  public static final double rollerSpeed = 0;

  public static final double upPosition = 0;
  public static final double downPosition = 0;
}
