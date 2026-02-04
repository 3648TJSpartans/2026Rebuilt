package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("LatestTargetObservation", latestTargetObservation);
    table.put("PoseObservations", poseObservations);
    table.put("TagIds", tagIds);
    table.put("CameraName", cameraName);
    table.put("Has Seen Target", hasSeenTarget);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
    poseObservations = table.get("PoseObservations", poseObservations);
    tagIds = table.get("TagIds", tagIds);
    cameraName = table.get("CameraName", cameraName);
    hasSeenTarget = table.get("Has Seen Target", hasSeenTarget);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.latestTargetObservation = this.latestTargetObservation;
    copy.poseObservations = this.poseObservations.clone();
    copy.tagIds = this.tagIds.clone();
    copy.cameraName = this.cameraName;
    copy.hasSeenTarget = this.hasSeenTarget;
    return copy;
  }
}
