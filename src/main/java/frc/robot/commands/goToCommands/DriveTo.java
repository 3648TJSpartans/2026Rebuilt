package frc.robot.commands.goToCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveTo extends Command {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Drive drive;

    public DriveTo(Drive drive, Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
        goToConstants.configurePID();
        this.robotPoseSupplier = robotPose;
        this.targetPoseSupplier = targetPose;
        this.drive = drive;
    }

    public DriveTo(Drive drive, Supplier<Pose2d> targetPose) {
        this(drive, drive::getPose, targetPose);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();
        // Gets the displacement vector -- if the robot goes absolutely the wrong way,
        // switch target pose and robot pose.
        Translation2d displacement = targetPose.getTranslation().minus(robotPose.getTranslation());
        // Defines translational speed the robot should go. displacement.getNorm is the
        // magnitude of the displacement.
        double driveSpeed = goToConstants.driveController.calculate(displacement.getNorm());
        // sets velocity to the normalized displacement vector(direction of
        // displacement) timesd the desired drive speed.
        Translation2d setVelocity = displacement.div(displacement.getNorm()).times(driveSpeed);

        // Gets the displacement angle -- if the robot rotates absolutely the wrong way,
        // switch target pose and robot pose.
        Rotation2d thetaDisplacement = targetPose.getRotation().minus(robotPose.getRotation());
        // Gets PID for thera displacement
        double thetaVelocity =
                goToConstants.thetaController.calculate(thetaDisplacement.getRadians());
        // runs velocities
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(-setVelocity.getX(),
                -setVelocity.getY(), -thetaVelocity, robotPose.getRotation()));

        // Lets Log stuff
        Logger.recordOutput("DriveTo/displacement", displacement);
        Logger.recordOutput("DriveTo/RobotPose", robotPose);
        Logger.recordOutput("DriveTo/Trajectory", displacement);
        Logger.recordOutput("DriveTo/TargetPose", targetPose);
        Logger.recordOutput("DriveTo/setDriveSpeed", driveSpeed);
        Logger.recordOutput("DriveTo/setDriveVelocity", setVelocity);
        Logger.recordOutput("DriveTo/thetaDifference", thetaDisplacement);
        Logger.recordOutput("DriveTo/thetaVelocity", thetaVelocity);
    }

    public boolean atGoal() {
        return goToConstants.driveController.atGoal() && goToConstants.thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }
}
