package frc.robot.subsystems.shiftTracker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShiftTracker extends SubsystemBase{
  private boolean firstTimeSlot;
  private boolean onShift;
  private double timeLeft;
  private double timeUntil;
  private double time;
  public ShiftTracker(){
    firstTimeSlot = false;
    onShift = false;
    time = 0.0;
  }

  @Override
  public void periodic(){
    boolean teleopEnabled = DriverStation.isTeleopEnabled();
    Logger.recordOutput("Subsystems/ShiftTracker/teleopEnabled", teleopEnabled);
    if(teleopEnabled){
      updateInputs();
    }
  }
  
  private void updateInputs(){
    time = DriverStation.getMatchTime();
    onShift = isOnShift();
    timeLeft = timeLeft();
    timeUntil = timeUntil();

    Logger.recordOutput("Subsystems/ShiftTracker/time", time);
    Logger.recordOutput("Subsystems/ShiftTracker/onShift", onShift);
    Logger.recordOutput("Subsystems/ShiftTracker/timeLeft", timeLeft);
    Logger.recordOutput("Subsystems/ShiftTracker/timeUntil", timeUntil);
  }

  private boolean isOnShift(){
    //Using an XOR gate
    //Returns true if we are the first time slot (true) and the shift is 1 or 3 (true). 
    //Returns true if we are the second time slot (false) and the shift is 2 or 4 (false). 
    //Returns false if we are the first time slot (true) and the shift is 2 or 4 (false). 
    //Returns false if we are the second time slot (false) and the shift is 1 or 3 (true). 
    return isEndgame() || !(firstTimeSlot ^ onShiftOneOrThree());
  }
  //
  private boolean onShiftOneOrThree(){
    //Shift 1
    if(time > ShiftTrackerConstants.shiftOneEnd){
      return true;
    }
    //Shift 2
    if(time > ShiftTrackerConstants.shiftTwoEnd){
      return false;
    }
    //Shift 3
    if(time >   ShiftTrackerConstants.shiftThreeEnd){
      return true;
    }
    return false;
  }

  private boolean isEndgame(){
    return time< ShiftTrackerConstants.endgameStart;
  }

  private double timeLeft(){
    if(!onShift){
      return 0.0;
    }
    return (time-30.0)%25.0;
  }

  private double timeUntil(){
    if(onShift){
      return 0.0;
    }
    return (time-30.0)%25.0;
  }



  public void setTimeSlot(boolean timeSlot){
    firstTimeSlot = timeSlot;
  }

  public boolean getOnShift(){
    return onShift;
  }

}
