package frc.robot.util.miscTunables.profiledPID;

import static frc.robot.util.TuningUpdater.TUNING_MODE;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import frc.robot.util.TunableNumber;
import frc.robot.util.TuningUpdater;
import java.util.function.Consumer;

public class TunableProfiledPIDController extends Tunable<ProfiledPIDController> {

  public TunableProfiledPIDController(String dashboardKey) {
    super(dashboardKey);
  }

  public TunableProfiledPIDController(String dashboardKey, ProfiledPIDController defaultValue) {
    super(dashboardKey, new EqualableProfiledPIDController(defaultValue));
  }

  public TunableProfiledPIDController(
      String dashboardKey, ProfiledPIDController defaultValue, Runnable update) {
    super(dashboardKey, new EqualableProfiledPIDController(defaultValue), update);
  }

  public TunableProfiledPIDController(
      String dashboardKey,
      ProfiledPIDController defaultValue,
      Consumer<ProfiledPIDController> updateConsumer) {
    super(dashboardKey, new EqualableProfiledPIDController(defaultValue), updateConsumer);
  }

  public TunableProfiledPIDController(
      String dashboardKey,
      ProfiledPIDController defaultValue,
      Runnable update,
      Consumer<ProfiledPIDController> updateConsumer) {
    super(dashboardKey, new EqualableProfiledPIDController(defaultValue), update, updateConsumer);
  }

  @Override
  protected void putDashboardValue(String key, ProfiledPIDController defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    final String dashboardKey = key;
    TunableNumber p = new TunableNumber(key + "/PID/P", defaultValue.getP(), defaultValue::setP);
    TunableNumber i = new TunableNumber(key + "/PID/I", defaultValue.getI(), defaultValue::setI);
    TunableNumber d = new TunableNumber(key + "/PID/D", defaultValue.getD(), defaultValue::setD);
    TunableNumber maxVelocity =
        new TunableNumber(
            key + "/constraints/maxVelocity",
            defaultValue.getConstraints().maxVelocity,
            (maxV) ->
                defaultValue.setConstraints(
                    new TrapezoidProfile.Constraints(
                        maxV, defaultValue.getConstraints().maxAcceleration)));
    TunableNumber maxAcceleration =
        new TunableNumber(
            key + "/constraints/maxAcceleration",
            defaultValue.getConstraints().maxAcceleration,
            (maxA) ->
                defaultValue.setConstraints(
                    new TrapezoidProfile.Constraints(
                        defaultValue.getConstraints().maxVelocity, maxA)));
    TunableNumber goal =
        new TunableNumber(key + "/goal", defaultValue.getGoal().position, defaultValue::setGoal);
    TunableNumber positionTolerance =
        new TunableNumber(
            key + "/tolerance", defaultValue.getPositionTolerance(), defaultValue::setTolerance);
  }

  @Override
  protected ProfiledPIDController getDashboardValue(
      String key, ProfiledPIDController defaultValue) {
    ProfiledPIDController out =
        TUNING_MODE
            ? new EqualableProfiledPIDController(
                SmartDashboard.getNumber(key + "/PID/p", defaultValue.getP()),
                SmartDashboard.getNumber(key + "/PID/i", defaultValue.getI()),
                SmartDashboard.getNumber(key + "/PID/d", defaultValue.getD()),
                defaultValue.getConstraints(),
                SmartDashboard.getNumber(key + "/period", defaultValue.getPeriod()))
            : defaultValue;

    return out;
  }

  @Override
  protected void logValue(
      String key,
      boolean TUNING_MODE,
      ProfiledPIDController value,
      ProfiledPIDController defaultValue) {
    return;
  }

  private static class EqualableProfiledPIDController extends ProfiledPIDController {
    public EqualableProfiledPIDController(
        double kP, double kI, double kD, TrapezoidProfile.Constraints constraints, double period) {
      super(kP, kI, kD, constraints, period);
    }

    public EqualableProfiledPIDController(ProfiledPIDController controller) {
      super(
          controller.getP(),
          controller.getI(),
          controller.getD(),
          controller.getConstraints(),
          controller.getPeriod());
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null || getClass() != obj.getClass()) {
        return false;
      }
      ProfiledPIDController other = (ProfiledPIDController) obj;
      return Double.compare(getP(), other.getP()) == 0
          && Double.compare(getI(), other.getI()) == 0
          && Double.compare(getD(), other.getD()) == 0
          && getConstraints().equals(other.getConstraints());
    }
  }
}
