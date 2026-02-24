package frc.robot.util;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// package org.littletonrobotics.frc2025.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
// import org.littletonrobotics.frc2025.FieldConstants;
// import
// org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.ModuleForce;
// import
// org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;
import frc.robot.PoseConstants;
import frc.robot.util.zoneCalc.Polygon;

public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? PoseConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? PoseConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static Translation2d[] apply(Translation2d[] poses) {
    Translation2d[] out = new Translation2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      out[i] = apply(poses[i]);
    }
    return out;
  }

  public static Pose2d[] apply(Pose2d[] poses) {
    Pose2d[] out = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      out[i] = apply(poses[i]);
    }
    return out;
  }

  public static Polygon apply(Polygon polygon) {
    return new Polygon(polygon.getName(), apply(polygon.getCorners()));
  }

  // @AutoLogOutput(key = "shouldFlip")
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
