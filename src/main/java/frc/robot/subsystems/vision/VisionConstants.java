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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final boolean usingMT1 = true;

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-three";
  public static String camera1Name = "N/A";
  public static int defualtPipeline = 0;
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0, 0.0, 0, new Rotation3d(0.0, 0, 0.0));
  public static Transform3d robotToCamera1 = new Transform3d(0, 0, 0, new Rotation3d(0.0, 0, 0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75; // TODO Updat this back

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation
  // data
  // available

  // Neural network values
  public static double nTargetHeight = 7.5; // Height of the center of the NN target in cm
  public static double nCameraHeight =
      11.5; // Height of the limelight camera from the *ground* in cm
  public static double nCameraAngle =
      0; // Angle of the limelight camera from perfectly horizontal in degrees
}
