package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.TunableBoolean;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

public class IntakeConstants {

  public static final MotorConfig hopperMotorConfig =
      new MotorConfig("Subsystems/Intake/HopperMotorIO")
          .motorCan(15)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final MotorConfig intakeRollerConfig =
      new MotorConfig("Subsystems/Intake/MotorIO")
          .motorCan(16)
          .p(0)
          .i(0)
          .d(0)
          .maxPower(.1)
          .minPower(-.1)
          .positionTolerance(0.0);

  public static final TunableNumber intakeRollerSpeed =
      new TunableNumber("Subsystems/Intake/intakeRollerSpeed", -0.25);
  public static final TunableNumber hopperSpeed =
      new TunableNumber("Subsystems/Intake/hopperSpeed", -.5);
  // Ideally the hopper is slowly spinning even when the robot isn't intaking or shooting
  // to push any balls in the hopper towards the shooter. It can't run fast or else
  // it will shred the balls.
  public static final TunableNumber hopperSlowSpeed =
      new TunableNumber("Subsystems/Intake/hopperSlowSpeed", 0);
  // This is the max speed that the robot will let itself run into a wall with while intake is down.
  // Faster than this and it automatically pulls the intake up.
  public static final TunableNumber maxIntakeSpeed =
      new TunableNumber("Subsystems/Intake/maxIntakeSpeed", 2);
  // This is how many seconds the robot will look into the future to decide when it will crash into
  // a wall. If the intake is going to hit a wall in less time than it can pull up the intake, it
  // attempts to bring it up.
  public static final TunableNumber pullUpTime =
      new TunableNumber("Subsystems/Intake/pullUpTime", 0.25);

  public static final TunableBoolean intakeProtected =
      new TunableBoolean("Subsystems/Intake/protected", true);

  public static final int solenoidChannel = 1;
  public static final Translation3d intakeOffset = new Translation3d(0.27, 0.0, .22);
  public static final Rotation3d intakeDownRotation = new Rotation3d(0, Math.PI / 2, 0);
  public static final Rotation3d intakeUpRotation = new Rotation3d();
  public static final Translation2d[] cornersDown =
      new Translation2d[] {
        new Translation2d(0.595665, -0.301625),
        new Translation2d(0.595665, 0.301625),
        new Translation2d(0.343, 0.301625),
        new Translation2d(0.343, -0.301625)
      };
  public static final Translation2d[] cornersUp =
      new Translation2d[] {
        new Translation2d(0.243, -0.301625),
        new Translation2d(0.243, 0.301625),
        new Translation2d(0.343, 0.301625),
        new Translation2d(0.343, -0.301625)
      };
}
