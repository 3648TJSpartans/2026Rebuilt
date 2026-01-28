package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.goToCommands.goToConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateTo extends Command {
  private final Drive drive;
  private Supplier<Double> targetRotation;

  /**
   * Constructor for RotateTo command.
   *
   * @param drive: A drive subsystem instance.
   * @param targetRotation: The target rotation angle in radians.
   */
  public RotateTo(Drive drive, Supplier<Double> targetRotation) {
    this.drive = drive;
    this.targetRotation = targetRotation;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Calculate the rotation displacement using our theta controller
    double rotationDisplacement =
        goToConstants.thetaController.calculate(Math.toRadians(-targetRotation.get()));
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationDisplacement);
    drive.runVelocity(speeds);

    // I love logging
    Logger.recordOutput("Commands/RotateTo/targetRotation", targetRotation.get());
    Logger.recordOutput("Commands/RotateTo/rotationDisplacement", rotationDisplacement);
  }

  @Override
  public boolean isFinished() {
    return goToConstants.thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
