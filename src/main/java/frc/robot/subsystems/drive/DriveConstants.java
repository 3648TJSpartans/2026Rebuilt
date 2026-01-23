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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

public class DriveConstants {
  public static final NavXComType navXComType = NavXComType.kUSB1;

  public static final double fieldRelativeMaxInputPercent = 1; // value between 0+ and 1,
  // allowing control maximum
  // input for field relative
  public static final double robotRelativeMaxInputPercent = .5; // value between 0+ and 1,
  // allowing control
  // maximum input for robot
  // relative

  public static final double maxSpeedMetersPerSec = 4; // 4.46 is max on rev product page, 2
  // is what it was before
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24.8); // TODO Update;
  public static final double wheelBase = Units.inchesToMeters(24.8);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Math.PI / 4);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(3 * Math.PI / 4);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(7 * Math.PI / 4);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(5 * Math.PI / 4);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  /*
   * Chassis Numbers 0- Tuning 1-Walter 2- Jebediah 3-
   *
   */

  // Encoder Offsets

  public static int chasNum = (int) new TunableNumber("Drive/ChassisNumber", 2).get();
  public static final double frontLeftExpectedZero = switch (chasNum) {
    case 0 -> 0.0;
    case 1 -> 1.757;
    case 2 -> 0.141;
    case 3 -> 0.0;
    default -> 0.0;
  };
  public static final double backLeftExpectedZero = switch (chasNum) {
    case 0 -> 0.0;
    case 1 -> 4.21;
    case 2 -> 2.957;
    case 3 -> 0.0;
    default -> 0.0;
  };
  public static final double frontRightExpectedZero = switch (chasNum) {
    case 0 -> 0.0;
    case 1 -> 4.08;
    case 2 -> 5.61;
    case 3 -> 0.0;
    default -> 0.0;
  };
  public static final double backRightExpectedZero = switch (chasNum) {
    case 0 -> 0.0;
    case 1 -> 4.16;
    case 2 -> 5.14;
    case 3 -> 0.0;
    default -> 0.0;
  };

  // Drive CAN IDs

  public static final int frontLeftDriveCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 8;
    case 2 -> 1;
    case 3 -> 0;
    default -> 0;
  };
  public static final int backLeftDriveCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 6;
    case 2 -> 3;
    case 3 -> 0;
    default -> 0;
  };
  public static final int frontRightDriveCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 2;
    case 2 -> 7;
    case 3 -> 0;
    default -> 0;
  };
  public static final int backRightDriveCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 4;
    case 2 -> 5;
    case 3 -> 0;
    default -> 0;
  };

  // Turn CAN IDs

  public static final int frontLeftTurnCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 7;
    case 2 -> 2;
    case 3 -> 0;
    default -> 0;
  };
  public static final int backLeftTurnCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 5;
    case 2 -> 4;
    case 3 -> 0;
    default -> 0;
  };
  public static final int frontRightTurnCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 1;
    case 2 -> 8;
    case 3 -> 0;
    default -> 0;
  };
  public static final int backRightTurnCanId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 3;
    case 2 -> 6;
    case 3 -> 0;
    default -> 0;
  };

  public static final int frontLeftTurnEncoderId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 3;
    case 2 -> 2;
    case 3 -> 0;
    default -> 0;
  };

  // Encoder CAN IDs

  public static final int backLeftTurnEncoderId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 2;
    case 2 -> 1;
    case 3 -> 0;
    default -> 0;
  };
  public static final int frontRightTurnEncoderId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 0;
    case 2 -> 3;
    case 3 -> 0;
    default -> 0;
  };
  public static final int backRightTurnEncoderId = switch (chasNum) {
    case 0 -> 0;
    case 1 -> 1;
    case 2 -> 0;
    case 3 -> 0;
    default -> 0;
  };

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction = 6.75; // L2 MKI4
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor
  // Rotations
  // ->
  // Wheel
  // Radians
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
  // ->
  // Wheel
  // Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.11241;
  public static final double driveKv = 0.13496;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM ->
  // Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 6.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -Math.PI; // Radians
  public static final double turnPIDMaxInput = Math.PI; // Radians
  public static final double turnTolerance = 0.01;

  // PathPlanner configuration
  public static final double robotMassKg = 45.3;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final double ppMaxSpeed = 4.2;
  public static final RobotConfig ppConfig = new RobotConfig(
      robotMassKg,
      robotMOI,
      new ModuleConfig(
          wheelRadiusMeters,
          ppMaxSpeed,
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          driveMotorCurrentLimit,
          1),
      moduleTranslations);
}
