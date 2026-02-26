// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.TunableBoolean;
import java.util.Arrays;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean DEFAULT_TUNING_MODE = true;

  public static final int kDriverControllerPort = 0;
  public static final int kCopilotControllerPort = 1;
  public static final int kTestControllerPort = 2;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double batteryGoodThreshold = 12.5;
  public static final double batteryWarningThreshold = 12.0;

  public static final TunableBoolean turretWorking =
      new TunableBoolean("Overrides/turretWorking", true);
  public static final TunableBoolean hoodWorking =
      new TunableBoolean("Overrides/hoodWorking", true);

  public static TunableBoolean doSmartShoot =
  new TunableBoolean("Overrides/doSmartShoot", false);


  public static final long usbFreeThreshold = 100000000; // 100MB File

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {}

  public static enum Status {
    OK,
    WARNING,
    ERROR,
    UNKNOWN
  }

  public static Status leastCommonStatus(Status a, Status b) {
    if (a == Status.OK && b == Status.WARNING) {
      return Status.WARNING;
    }
    if (a == Status.WARNING && b == Status.ERROR) {
      return Status.ERROR;
    }
    if (a == Status.OK && b == Status.ERROR) {
      return Status.ERROR;
    }
    return a;
  }

  public static Status leastCommonStatus(Status... statuses) {

    return leastCommonStatus(
        statuses[0], leastCommonStatus(Arrays.copyOfRange(statuses, 1, statuses.length)));
  }

  public static String usbPath = "/media/sdb1";
}
