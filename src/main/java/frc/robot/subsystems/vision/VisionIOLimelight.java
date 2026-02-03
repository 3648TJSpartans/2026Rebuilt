// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.defualtPipeline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LimelightHelpers;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleArraySubscriber botpose_targetSpaceSubscriber;
  private final String name;
  private final String outputName;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    outputName = name + "/tag_relative";
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    botpose_targetSpaceSubscriber =
        table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[6]);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public double getTx() {
    var table = NetworkTableInstance.getDefault().getTable(name);
    return table.getDoubleTopic("tx").subscribe(0.0).getAsDouble();
  }

  public double getTy() {
    var table = NetworkTableInstance.getDefault().getTable(name);
    return table.getDoubleTopic("tync").subscribe(0.0).getAsDouble();
  }

  // If we're always gettign 0, the error is in here
  @Override
  public Pose2d getTagRelativePose() {
    double[] tableValues = botpose_targetSpaceSubscriber.get();
    return new Pose2d(
        tableValues[2], -tableValues[0], new Rotation2d(Units.degreesToRadians(tableValues[4])));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 500;
    inputs.cameraName = name;
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(name, rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (mt2 == null) {
      doRejectUpdate = true;
    } else if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {

      poseObservations.add(mt2.getAsObservartion());
    }
    doRejectUpdate = false;
    if (VisionConstants.usingMT1) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      if (mt1 == null) {
        doRejectUpdate = true;
      } else if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {

        poseObservations.add(mt1.getAsObservartion());
      }
    }
    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  @Override
  public void setPipeline(int pipeline) {
    // var table = NetworkTableInstance.getDefault().getTable(name);
    // table.getEntry("pipeline").setNumber(pipeline);
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  @Override
  public int getPipeline() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(name);
  }

  @Override
  public void resetPipeline() {
    setPipeline(defualtPipeline);
  }
}
