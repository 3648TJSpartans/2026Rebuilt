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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Status;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FFCharacterizationCmd;
import frc.robot.commands.goToCommands.DriveTo;
import frc.robot.commands.goToCommands.DriveToTag;
import frc.robot.commands.goToCommands.goToConstants;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;
import frc.robot.commands.ledCommands.ShiftOffLEDCommand;
import frc.robot.commands.ledCommands.ShiftOnLEDCommand;
import frc.robot.commands.ledCommands.StatusCheckLEDCommand;
import frc.robot.commands.trajectoryCommands.RunDynamicTrajectory;
import frc.robot.commands.trajectoryCommands.RunTrajectoryCmd;
import frc.robot.commands.trajectoryCommands.TrajectoryConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.LoggedAnalogEncoder;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMK4Spark;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.shiftTracker.ShiftTracker;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Neural;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RangeCalc;
import frc.robot.util.TunableNumber;
import frc.robot.util.TuningUpdater;
import frc.robot.util.motorUtil.MotorIO;
import frc.robot.util.statusableUtils.GenericStatusable;
import frc.robot.util.statusableUtils.StatusLogger;
import frc.robot.util.trajectorySolver.TrajectoryLogger;
import java.io.File;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  // private final CompressorIO m_compressor;
  private final LedSubsystem m_leds;
  private final Vision m_vision;
  private final Neural m_neural;
  private final Hood m_hood;
  private final ShiftTracker m_shiftTracker;
  private final Climber m_climber;
  private boolean override;
  private boolean endgameClosed = true;
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Kicker m_kicker;
  private final Intake m_intake;
  private final Hopper m_hopper;
  private final TrajectoryLogger m_trajectoryLogger;
  private final GenericStatusable m_usbStatus;
  private final GenericStatusable m_batteryStatus;
  private final StatusLogger m_statusLogger;
  // Controller
  private final CommandXboxController m_driveController =
      new CommandXboxController(Constants.kDriverControllerPort);
  private final CommandXboxController m_copilotController =
      new CommandXboxController(Constants.kCopilotControllerPort);
  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.kTestControllerPort);
  private final CommandXboxController m_test3Controller = new CommandXboxController(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);
  private final LoggedNetworkNumber endgameAlert3 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #3", 5.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leds = new LedSubsystem();
    m_shiftTracker = new ShiftTracker();
    m_climber = new Climber();
    m_hood = new Hood();
    m_shooter = new Shooter();
    m_kicker = new Kicker();
    m_intake = new Intake();
    m_hopper = new Hopper();
    // m_compressor = new CompressorIO("Compressor");
    m_usbStatus =
        new GenericStatusable(
            () -> {
              File drive = new File(Constants.usbPath);
              if (drive.exists() && drive.isDirectory() && drive.canWrite()) {
                double freeSpace = drive.getUsableSpace();
                if (freeSpace < Constants.usbFreeThreshold) {
                  Logger.recordOutput("Debug/USB/warning", "USB near full");
                  Logger.recordOutput("Debug/USB/freeSpace", freeSpace);
                  return Status.WARNING;
                }
                Logger.recordOutput("Debug/USB/warning", "N/A");
                return Status.OK;
              }
              Logger.recordOutput("Debug/USB/warning", "not found");
              return Status.WARNING;
            },
            "USB",
            5);

    m_batteryStatus =
        new GenericStatusable(
            () -> {
              double voltage = RobotController.getBatteryVoltage();
              if (voltage > Constants.batteryGoodThreshold) {

                return Status.OK;
              }
              if (voltage > Constants.batteryWarningThreshold) {
                Logger.recordOutput("Debug/Battery/voltage", voltage);
                return Status.WARNING;
              }
              Logger.recordOutput("Debug/Battery/voltage", voltage);
              return Status.ERROR;
            },
            "Battery",
            10);

    Logger.recordOutput("Utils/Poses/shouldFlip", AllianceFlipUtil.shouldFlip());
    Logger.recordOutput("Override", override);
    override = false;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOMK4Spark(0),
                new ModuleIOMK4Spark(1),
                new ModuleIOMK4Spark(2),
                new ModuleIOMK4Spark(3));
        // new Drive(
        //     new GyroIONavX(),
        //     new ModuleIO() {},
        //     new ModuleIO() {},
        //     new ModuleIO() {},
        //     new ModuleIO() {});

        // To change number of limelights, just add or delete IOs in the
        // parameters
        // Make sure camera name match in the coprocessor!
        switch (DriveConstants.chasNum) {
          case 3:
            m_vision =
                new Vision(
                    m_drive::addVisionMeasurement,
                    m_drive::addTargetSpaceVisionMeasurement,
                    // new
                    // VisionIOLimelight(VisionConstants.camera0Name,
                    // m_drive::getRotation),
                    new VisionIOLimelight("limelight-fourb", m_drive::getRotation));
            break;
          case 1:
            m_vision =
                new Vision(
                    m_drive::addVisionMeasurement,
                    m_drive::addTargetSpaceVisionMeasurement,
                    // new
                    // VisionIOLimelight(VisionConstants.camera1Name,
                    // m_drive::getRotation),
                    new VisionIOLimelight(VisionConstants.camera0Name, m_drive::getRotation),
                    new VisionIOLimelight("limelight-fourone", m_drive::getRotation),
                    new VisionIOLimelight("limelight-fourthr", m_drive::getRotation));
            break;
          default:
            m_vision =
                new Vision(m_drive::addVisionMeasurement, m_drive::addTargetSpaceVisionMeasurement);
            break;
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        m_vision =
            new Vision(
                m_drive::addVisionMeasurement, m_drive::addTargetSpaceVisionMeasurement
                // new VisionIOLimelight(VisionConstants.camera0Name,
                // m_drive::getRotation),
                // new VisionIOLimelight(VisionConstants.camera1Name,
                // m_drive::getRotation)
                );
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                m_drive::addTargetSpaceVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, m_drive::getRotation));
        break;
    }

    m_turret = new Turret(m_drive::getPose, m_drive::getVelocity);
    // TODO update as subsystems are made
    m_trajectoryLogger =
        new TrajectoryLogger(
            () -> Units.degreesToRadians(80),
            () -> m_turret.getTurretRotation().getRadians(),
            m_shooter::getVelocity,
            () -> 2.0,
            () -> m_turret.getTurretFieldPose().getTranslation(),
            m_turret::getTurretTranslationalVelocity);

    m_neural = new Neural(m_drive::getPose);

    m_statusLogger =
        new StatusLogger(
            m_climber,
            m_hood,
            m_shooter,
            m_kicker,
            m_intake,
            // m_compressor,
            m_hopper,
            m_drive,
            m_vision,
            m_usbStatus,
            m_batteryStatus);

    configureAutos();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutoChooser();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureAutos() {}

  private void configureButtonBindings() {
    // configureAutos();
    configureLeds();
    configureAutoChooser();
    configureSimpleMotor();
    configureDrive();
    configureShooter();
    configureAlerts();
    // configureClimber();
    configureIntake();
    configureHopper();
    configureTurret();
    configureHood();
    configureKicker();
    Command updateCommand =
        new InstantCommand(
                () -> {
                  MotorIO.reconfigureMotors();
                  goToConstants.configurePID();
                })
            .ignoringDisable(true);
    m_copilotController.rightTrigger().onTrue(updateCommand);
    m_testController
        .povUp()
        .onTrue(new InstantCommand(() -> LoggedAnalogEncoder.updateZeros()).ignoringDisable(true));
    new Trigger(() -> DriverStation.isEnabled() && TuningUpdater.TUNING_MODE).onTrue(updateCommand);
    m_driveController.rightTrigger().onTrue(new InstantCommand(this::toggleOverride));

    /*
     * m_led.setLedPattern(LedConstants.elevatorHeight, m_led.elevatorBuffer);
     * m_led.setLedPattern(LedConstants.teal, m_led.leftGuideBuffer);
     * m_led.setLedPattern(LedConstants.yellow, m_led.rightGuideBuffer);
     */
  }

  private void configureAlerts() {
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && m_shiftTracker.timeUntil() < 5.0
                    && m_shiftTracker.timeUntil() > 0.0)
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.75)
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly()
                .withTimeout(5)

            // .beforeStarting(() -> leds.endgameAlert = true)
            // .finallyDo(() -> leds.endgameAlert = false)
            );

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .andThen(Commands.waitSeconds(4.75))
                .repeatedly()
                .withTimeout(15)

            // .beforeStarting(() -> leds.endgameAlert = true)
            // .finallyDo(() -> leds.endgameAlert = false)
            );
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(8)
            // .beforeStarting(() -> leds.endgameAlert = true)
            // .finallyDo(() -> leds.endgameAlert = false)
            );
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.3))
                .repeatedly()
                .withTimeout(10)
            // .beforeStarting(() -> leds.endgameAlert = true)
            // .finallyDo(() -> leds.endgameAlert = false)
            );
    // Countdown
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert3.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.8)
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly()
                .withTimeout(5)
            // .beforeStarting(() -> leds.endgameAlert = true)
            // .finallyDo(() -> leds.endgameAlert = false)
            );
  }

  private void configureKicker() {
    m_kicker.setDefaultCommand(
        Commands.run(
            () -> m_kicker.setPower(MathUtil.applyDeadband(m_testController.getRightY(), 0.1)),
            m_kicker));
  }

  private void configureTurret() {
    // m_turret.setDefaultCommand(new TurretFollowCmd(m_turret,()-> new Pose2d(1,1,
    // new
    // Rotation2d())));
    // m_testController.a().onTrue(new InstantCommand(m_turret::setZeroHeading));
    TunableNumber turretSetpoint = new TunableNumber("Subsystems/Turret/testSetpoint", 0.5);
    // m_testController
    //     .rightBumper()
    //     .onTrue(Commands.runOnce(() -> m_turret.setPower(turretPower.get()), m_turret))
    //     .onFalse(new InstantCommand(m_turret::stop, m_turret));
    // m_testController
    //     .leftBumper()
    //     .onTrue(Commands.runOnce(() -> m_turret.setPower(-turretPower.get()), m_turret))
    //     .onFalse(new InstantCommand(m_turret::stop, m_turret));

    // TunableNumber setPose = new TunableNumber("Subsystems/Turret/testSetPose", 0.0);
    // m_testController
    //     .rightTrigger()
    //     .whileTrue(Commands.run(() -> m_turret.setRotation(new Rotation2d(setPose.get()))));

    m_turret.setDefaultCommand(
        Commands.run(
            () ->
                m_turret.setPower(MathUtil.applyDeadband(m_test3Controller.getLeftY(), 0.1) / 10.0),
            m_turret));
    m_test3Controller
        .a()
        .whileTrue(
            Commands.run(
                    () -> {
                      m_turret.pointAt(TrajectoryConstants.hubPose.toTranslation2d());
                    },
                    m_turret)
                .finallyDo(() -> m_turret.stop()));
    m_test3Controller.b().onTrue(Commands.runOnce(m_turret::setZeroHeading, m_turret));

    // Random rand = new Random();
    // TunableNumber targetX =
    // new TunableNumber("Subsystems/Turret/testTargeting/x", rand.nextDouble() *
    // 5);
    // TunableNumber targetY =
    // new TunableNumber("Subsystems/Turret/testTargeting/y", rand.nextDouble() *
    // 5);

    // m_testController
    //     .b()
    //     .whileTrue(
    //         Commands.run(() -> m_turret.pointAt(TrajectoryConstants.hubPose.toTranslation2d())));

    RunTrajectoryCmd dynamicTrajectory =
        new RunDynamicTrajectory(
            m_turret,
            m_shooter,
            m_hood,
            () -> TrajectoryConstants.overhangHeight,
            () -> TrajectoryConstants.overhangAspect,
            () -> TrajectoryConstants.hubPose,
            () -> RangeCalc.inShootingRange(m_drive.getPose()),
            () -> m_drive.getTilt(),
            () -> m_shiftTracker.timeLeft(),
            () -> m_shiftTracker.timeUntil());
    TunableNumber aspect = new TunableNumber("Trajectory/testTrajHeight", 1.5);
    RunTrajectoryCmd dynamicTestTrajectory =
        new RunDynamicTrajectory(
            m_turret,
            m_shooter,
            m_hood,
            () -> aspect.get(),
            () -> .5,
            () -> TrajectoryConstants.hubPose,
            () -> RangeCalc.inShootingRange(m_drive.getPose()),
            () -> m_drive.getTilt(),
            () -> m_shiftTracker.timeLeft(),
            () -> m_shiftTracker.timeUntil());
    // m_testController.povLeft().whileTrue(dynamicTrajectory);
    // m_driveController.povLeft().whileTrue(dynamicTrajectory);

    RunTrajectoryCmd feedAlliance =
        new RunDynamicTrajectory(
            m_turret,
            m_shooter,
            m_hood,
            () ->
                switch (RangeCalc.zoneCalc(m_drive.getPose())) {
                  case 1 -> PoseConstants.overhangMiddle;
                  default -> PoseConstants.overhangSide;
                },
            () -> .5,
            () ->
                switch (RangeCalc.zoneCalc(m_drive.getPose())) {
                  case 0 -> PoseConstants.feedRight;
                  case 1 -> PoseConstants.feedMiddle;
                  case 2 -> PoseConstants.feedLeft;
                  default -> PoseConstants.feedMiddle;
                },
            () -> !RangeCalc.inShootingRange(m_drive.getPose()),
            () -> m_drive.getTilt(),
            () -> m_shiftTracker.timeUntil() - TrajectoryConstants.allianceFeedingCutoffTime,
            () -> m_shiftTracker.timeLeft());

    m_test3Controller
        .y()
        .whileTrue(
            dynamicTestTrajectory.alongWith(
                Commands.run(
                        () -> {
                          if (m_shooter.speedInTolerance()) {
                            m_kicker.setPower(1.0);
                            m_hopper.setPower(-.5);
                          }
                        },
                        m_kicker)
                    .finallyDo(
                        () -> {
                          m_kicker.stop();
                          m_hopper.stop();
                        })));

    // m_kicker.setDefaultCommand(
    //     Commands.run(
    //         () -> m_kicker.runExceptSensor(ShooterConstants.kickerSlowSpeed.get()), m_kicker));
    // new Trigger(() -> dynamicTrajectory.ready())
    //     .whileTrue(Commands.run(() -> m_kicker.setSpeed(ShooterConstants.kickerSpeed.get())))
    //     .whileTrue(Commands.run(() -> m_hopper.setSpeed(IntakeConstants.hopperSpeed.get())));
  }

  private void configureShooter() {
    TunableNumber shootSpeed = new TunableNumber("Subsystems/Shooter/testShootRPM", 500);
    m_testController
        .x()
        .whileTrue(
            Commands.run(
                    () -> {
                      m_shooter.runFFVelocity(shootSpeed.get());
                      if (m_shooter.speedInTolerance()) {
                        m_kicker.setPower(ShooterConstants.kickerSpeed.get());
                        m_hopper.setPower(IntakeConstants.hopperSpeed.get());
                      } else {
                        m_kicker.stop();
                        m_hopper.stop();
                      }
                    },
                    m_shooter,
                    m_kicker,
                    m_hopper)
                .finallyDo(
                    () -> {
                      m_shooter.stop();
                      m_kicker.stop();
                    }));
    // m_testController
    //     .leftTrigger()
    //     .onTrue(Commands.runOnce(() -> m_kicker.setPower(ShooterConstants.kickerSpeed.get())))
    //     .onFalse(Commands.runOnce(() -> m_kicker.stop()));
    m_shooter.setDefaultCommand(
        Commands.run(
            () -> m_shooter.setPower(MathUtil.applyDeadband(m_test3Controller.getRightY(), 0.1)),
            m_shooter));
  }

  public void configureAutoChooser() {

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter simple FF Identification",
        FFCharacterizationCmd.characterizeSystem(
                m_shooter,
                speed -> m_shooter.runCharacterization(speed),
                m_shooter::getFFCharacterizationVelocity)
            .finallyDo(m_shooter::stop));
  }

  public void configureSimpleMotor() {
    // Command simpleForward =
    // new SimpleMotorCmd(m_simpleMotor, SimpleMotorConstants.speed1);
    // Command simpleBackward =
    // new SimpleMotorCmd(m_simpleMotor, -SimpleMotorConstants.speed1);

    // m_copilotController.leftBumper().whileTrue(simpleBackward);
    // m_copilotController.rightBumper().whileTrue(simpleForward);
  }

  public void configureIntake() {

    m_testController.povDown().onTrue(Commands.runOnce(() -> m_intake.setSolenoid(true)));
    m_testController.povUp().onTrue(Commands.runOnce(() -> m_intake.setSolenoid(false)));
    m_testController
        .y()
        .whileTrue(Commands.run(() -> m_intake.setRollers(IntakeConstants.intakeRollerSpeed.get())))
        .onFalse(Commands.runOnce(() -> m_intake.stopRollers()));

    m_intake.setDefaultCommand(
        Commands.run(
                () -> m_intake.setRollers(MathUtil.applyDeadband(m_testController.getLeftY(), 0.1)),
                m_intake)
            .finallyDo(m_intake::stopRollers));

    m_driveController
        .y()
        .whileTrue(Commands.runOnce(() -> m_intake.setSolenoidAndRollerDown()))
        .onFalse(Commands.runOnce(() -> m_intake.setSolenoidAndRollerUp()));
  }

  public void configureHopper() {

    m_testController
        .a()
        .onTrue(Commands.runOnce(() -> m_hopper.setPower(IntakeConstants.hopperSpeed.get())))
        .onFalse(Commands.runOnce(() -> m_hopper.setPower(IntakeConstants.hopperSlowSpeed.get())));
    m_testController
        .povLeft()
        .whileTrue(
            Commands.run(() -> m_hopper.setPower(-IntakeConstants.hopperSpeed.get()), m_hopper))
        .onFalse(Commands.runOnce(() -> m_hopper.setPower(IntakeConstants.hopperSlowSpeed.get())));
    m_testController
        .povRight()
        .whileTrue(
            Commands.run(() -> m_hopper.setPower(-IntakeConstants.hopperSpeed.get()), m_hopper))
        .onFalse(Commands.runOnce(() -> m_hopper.setPower(IntakeConstants.hopperSlowSpeed.get())));

    m_hopper.setDefaultCommand(
        Commands.run(
            () ->
                m_hopper.setPower(
                    -MathUtil.applyDeadband(m_testController.getRightTriggerAxis(), 0.05) / 2.0),
            m_hopper));
  }

  public void configureLeds() {

    // This code changes LED patterns when the robot is in auto or teleop.
    // It can be manipulated for your desires

    // Command AutoLED = new AutoLEDCommand(m_leds);
    // Command TeleopLED = new TeleopLEDCommand(m_leds);

    Trigger shiftTrigger = new Trigger(() -> m_shiftTracker.getOnShift());
    shiftTrigger.onTrue(new ShiftOnLEDCommand(m_leds, m_shiftTracker, LedConstants.green));
    shiftTrigger.onFalse(new ShiftOffLEDCommand(m_leds, m_shiftTracker, LedConstants.red));

    WrapperCommand statusCheck =
        new StatusCheckLEDCommand(m_leds, m_statusLogger.getStatuses()).ignoringDisable(true);

    m_leds.setDefaultCommand(statusCheck);
    m_statusLogger.setDefaultCommand(
        Commands.run(() -> m_statusLogger.logStatuses(), m_statusLogger));
    // new Trigger(() -> DriverStation.isDisabled()).whileTrue();
    // Trigger autonomous = new Trigger(() -> DriverStation.isAutonomousEnabled());
    // Trigger teleop = new Trigger(() -> DriverStation.isTeleopEnabled());

    // autonomous.onTrue(AutoLED);
    // teleop.onTrue(TeleopLED);
    /* */
  }

  private void configureHood() {
    m_test3Controller
        .povUp()
        .whileTrue(Commands.run(() -> m_hood.setPower(0.05), m_hood).finallyDo(m_hood::stop));
    m_test3Controller
        .povDown()
        .whileTrue(Commands.run(() -> m_hood.setPower(-0.05), m_hood).finallyDo(m_hood::stop));
    TunableNumber testPose = new TunableNumber("Subsystems/Hood/testPosition", 0.25);
    m_test3Controller
        .leftBumper()
        .whileTrue(
            Commands.run(() -> m_hood.setPosition(testPose.get()), m_hood).finallyDo(m_hood::stop));
    m_test3Controller
        .rightBumper()
        .whileTrue(Commands.run(() -> m_hood.setPosition(0.5), m_hood).finallyDo(m_hood::stop));
  }

  public void configureDrive() {
    // Default command, normal field-relative drive/
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driveController.getLeftY(),
            () -> -m_driveController.getLeftX(),
            () -> -m_driveController.getRightX(),
            m_driveController.leftBumper()));

    // Lock to nearest 45° when A button is held
    Rotation2d[] lockpoints = {
      new Rotation2d(Math.PI / 4),
      new Rotation2d(3 * Math.PI / 4),
      new Rotation2d(-3 * Math.PI / 4),
      new Rotation2d(-Math.PI / 4),
    };

    m_driveController
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> -m_driveController.getLeftY(),
                () -> -m_driveController.getLeftX(),
                () -> {
                  Rotation2d driveRotation = m_drive.getRotation();
                  double smallestDiff = Double.MAX_VALUE;
                  Rotation2d closestLockpoint = new Rotation2d(0.0);
                  for (Rotation2d lockpoint : lockpoints) {
                    double diff = Math.abs(driveRotation.minus(lockpoint).getRadians());
                    if (diff < smallestDiff) {
                      smallestDiff = diff;
                      closestLockpoint = lockpoint;
                    }
                  }
                  return closestLockpoint;
                }));

    // Switch to X pattern when X button is pressed
    m_driveController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when A button is pressed
    m_driveController
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
                            new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
                    m_drive)
                .ignoringDisable(true));
    m_driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
                            new Pose2d(
                                m_drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    m_drive)
                .ignoringDisable(true));

    m_driveController
        .y()
        .onTrue(Commands.runOnce(() -> m_vision.setPipeline(1, 0)))
        .whileTrue(
            new WaitUntilCommand(() -> m_vision.getPipeline(0) == 1 && m_neural.isPoseDetected())
                .andThen(
                    Commands.runOnce(m_neural::updateSavedPose)
                        .andThen(new DriveTo(m_drive, () -> m_neural.getSavedPose()))))
        .onFalse(Commands.runOnce(() -> m_vision.setPipeline(0, 0)));

    Command driveTest = new DriveTo(m_drive, () -> PoseConstants.examplePose);
    Pose2d alignOffsetRight = new Pose2d(new Translation2d(-.75, -.17), new Rotation2d(0));
    Pose2d alignOffsetLeft = new Pose2d(new Translation2d(-.75, .17), new Rotation2d(0));
    Command alignToTagRight =
        new DriveToTag(m_drive, m_drive::getTargetSpacePose, () -> alignOffsetRight);
    Command alignToTagLeft =
        new DriveToTag(m_drive, m_drive::getTargetSpacePose, () -> alignOffsetLeft);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void toggleOverride() {
    override = !override;
    Logger.recordOutput("Override", override);
  }

  private void configureCommandGroups() {}

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          m_driveController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          m_driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }
}
