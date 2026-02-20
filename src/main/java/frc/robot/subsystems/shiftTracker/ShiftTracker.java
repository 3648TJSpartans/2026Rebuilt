package frc.robot.subsystems.shiftTracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShiftTracker extends SubsystemBase {
  private boolean onShift;
  private double timeLeft;
  private double timeUntil;
  private double time;
  private boolean firstTimeSlot;

  public ShiftTracker() {
    onShift = false;
    time = 0.0;
    firstTimeSlot = false;
  }

  @Override
  public void periodic() {
    boolean teleopEnabled = DriverStation.isTeleopEnabled();
    Logger.recordOutput("Subsystems/ShiftTracker/teleopEnabled", teleopEnabled);
    if (teleopEnabled) {
      updateInputs();
      return;
    }
    // This code is used to make sure the robot doesn't stop itself when testing, or if we were to
    // implement this wrong in autos. If it cuases error, comment it out.
    overrideInputs();
  }

  private void updateInputs() {
    time = DriverStation.getMatchTime();
    onShift = isOnShift();
    timeLeft = timeLeft();
    timeUntil = timeUntil();

    Logger.recordOutput("Subsystems/ShiftTracker/time", time);
    Logger.recordOutput("Subsystems/ShiftTracker/onShift", onShift);
    Logger.recordOutput("Subsystems/ShiftTracker/timeLeft", timeLeft);
    Logger.recordOutput("Subsystems/ShiftTracker/timeUntil", timeUntil);
  }

  private void overrideInputs() {
    time = 0;
    onShift = true;
    timeLeft = 25.0;
    timeUntil = 0.0;

    Logger.recordOutput("Subsystems/ShiftTracker/time", time);
    Logger.recordOutput("Subsystems/ShiftTracker/onShift", onShift);
    Logger.recordOutput("Subsystems/ShiftTracker/timeLeft", timeLeft);
    Logger.recordOutput("Subsystems/ShiftTracker/timeUntil", timeUntil);
  }

  public boolean isOnShift() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  // This requires input from the copilot, so it isn't as good in theory as the
  // above solution. However, we can't verify that the above solution works because
  // it requires data from the FMS, which we don't have unless we're in competition.
  public boolean onShiftDeprecated() {
    return isEndgame() || !(firstTimeSlot ^ onShiftOneOrThree());
  }

  //
  private boolean onShiftOneOrThree() {
    // Shift 1
    if (time > ShiftTrackerConstants.shiftOneEnd) {
      return true;
    }
    // Shift 2
    if (time > ShiftTrackerConstants.shiftTwoEnd) {
      return false;
    }
    // Shift 3
    if (time > ShiftTrackerConstants.shiftThreeEnd) {
      return true;
    }
    return false;
  }

  private boolean isEndgame() {
    return time < ShiftTrackerConstants.endgameStart;
  }

  public double timeLeft() {
    if (!onShift) {
      return 0.0;
    }
    return (time - 30.0) % 25.0;
  }

  public double timeUntil() {
    if (onShift) {
      return 0.0;
    }
    return (time - 30.0) % 25.0;
  }

  // Deprecated
  public void setTimeSlot(boolean timeSlot) {
    firstTimeSlot = timeSlot;
  }

  public boolean getOnShift() {
    return onShift;
  }
}
/*  make the controller vibrate evey second  */
