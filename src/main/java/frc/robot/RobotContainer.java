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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FFCharacterizationCmd;
import frc.robot.commands.exampleSubsystemCommands.ExampleMotorCmd;
import frc.robot.commands.goToCommands.DriveTo;
import frc.robot.commands.goToCommands.DriveToTag;
import frc.robot.commands.goToCommands.goToConstants;
import frc.robot.commands.goToCommands.goToConstants.PoseConstants;
import frc.robot.commands.ledCommands.AutoLEDCommand;
import frc.robot.commands.ledCommands.TeleopLEDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.LoggedAnalogEncoder;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMK4Spark;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.exampleMotorSubsystem.ExampleMotorSubsystem;
import frc.robot.subsystems.exampleMotorSubsystem.ExampleMotorSubsystemConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.simpleMotor.SimpleMotor;
import frc.robot.subsystems.simpleMotor.SimpleMotorSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;
import frc.robot.util.motorUtil.MotorIO;
import frc.robot.util.motorUtil.RelEncoderSparkMax;
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
  private final SimpleMotor m_simpleMotor;
  private final LedSubsystem m_leds;
  private final Vision m_vision;
  private final ExampleMotorSubsystem m_exampleMotorSubsystem;
  private final RelEncoderSparkMax m_exampleFlywheel;
  private boolean override;
  private boolean endgameClosed = true;

  // Controller
  private final CommandXboxController m_driveController =
      new CommandXboxController(Constants.kDriverControllerPort);
  private final CommandXboxController m_copilotController =
      new CommandXboxController(Constants.kCopilotControllerPort);
  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.kTestControllerPort);
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
    m_simpleMotor = new SimpleMotor(new SimpleMotorSparkMax());
    m_leds = new LedSubsystem();
    m_exampleMotorSubsystem = new ExampleMotorSubsystem();
    // CAN 10
    m_exampleFlywheel =
        new RelEncoderSparkMax(new MotorConfig("Flywheel").motorCan(10).Ks(0.0).Kv(0.0));
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

        // To change number of limelights, just add or delete IOs in the
        // parameters
        // Make sure camera name match in the coprocessor!
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                m_drive::addTargetSpaceVisionMeasurement,
                // new
                // VisionIOLimelight(VisionConstants.camera1Name,
                // m_drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera0Name, m_drive::getRotation));
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
    configureFlywheel();
    // configureExampleSubsystem();
    Command updateCommand =
        new InstantCommand(
                () -> {
                  MotorIO.reconfigureMotors();
                  goToConstants.configurePID();
                  LoggedAnalogEncoder.updateZeros();
                })
            .ignoringDisable(true);
    m_copilotController.rightTrigger().onTrue(updateCommand);

    new Trigger(DriverStation::isEnabled).onTrue(updateCommand);
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
        "Flywheel simple FF IDentification",
        FFCharacterizationCmd.characterizeSystem(
            m_exampleFlywheel,
            speed -> m_exampleFlywheel.runCharacterization(speed),
            m_exampleFlywheel::getFFCharacterizationVelocity));
  }

  public void configureFlywheel() {

    TunableNumber flywheelSpeed = new TunableNumber("MotorIOs/Flywheel/commandSpeed", -1000.0);
    m_copilotController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> m_exampleFlywheel.runFFVelocity(flywheelSpeed.get()), m_exampleFlywheel))
        .onFalse(Commands.runOnce(() -> m_exampleFlywheel.stop(), m_exampleFlywheel));
  }

  public void configureSimpleMotor() {
    // Command simpleForward =
    // new SimpleMotorCmd(m_simpleMotor, SimpleMotorConstants.speed1);
    // Command simpleBackward =
    // new SimpleMotorCmd(m_simpleMotor, -SimpleMotorConstants.speed1);

    // m_copilotController.leftBumper().whileTrue(simpleBackward);
    // m_copilotController.rightBumper().whileTrue(simpleForward);
  }

  public void configureLeds() {

    // This code changes LED patterns when the robot is in auto or teleop.
    // It can be manipulated for your desires

    Command AutoLED = new AutoLEDCommand(m_leds);
    Command TeleopLED = new TeleopLEDCommand(m_leds);

    Trigger autonomous = new Trigger(() -> DriverStation.isAutonomousEnabled());
    Trigger teleop = new Trigger(() -> DriverStation.isTeleopEnabled());

    autonomous.onTrue(AutoLED);
    teleop.onTrue(TeleopLED);
  }

  public void configureDrive() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driveController.getLeftY(),
            () -> -m_driveController.getLeftX(),
            () -> -m_driveController.getRightX(),
            m_driveController.leftBumper()));

    // Lock to 0° when A button is held
    // m_driveController
    // .b()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // m_drive,
    // () -> m_driveController.getLeftY(),
    // () -> m_driveController.getLeftX(),
    // () -> new Rotation2d()));

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
    Command driveTest = new DriveTo(m_drive, () -> PoseConstants.examplePose);
    Pose2d alignOffsetRight = new Pose2d(new Translation2d(-.75, -.17), new Rotation2d(0));
    Pose2d alignOffsetLeft = new Pose2d(new Translation2d(-.75, .17), new Rotation2d(0));
    Command alignToTagRight =
        new DriveToTag(m_drive, m_drive::getTargetSpacePose, () -> alignOffsetRight);
    Command alignToTagLeft =
        new DriveToTag(m_drive, m_drive::getTargetSpacePose, () -> alignOffsetLeft);
    m_driveController
        .rightTrigger()
        .whileTrue(
            new DriveTo(m_drive, () -> new Pose2d(1.5, 0.5, new Rotation2d(Math.PI)))
                .alongWith(new InstantCommand(() -> goToConstants.configurePID())));
    m_driveController
        .leftTrigger()
        .whileTrue(new DriveTo(m_drive, () -> new Pose2d(0.0, 0.0, new Rotation2d())));
  }

  public void configureExampleSubsystem() {
    Command motorCommand =
        new ExampleMotorCmd(m_exampleMotorSubsystem, ExampleMotorSubsystemConstants.power);
    m_testController.leftBumper().whileTrue(motorCommand);
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
