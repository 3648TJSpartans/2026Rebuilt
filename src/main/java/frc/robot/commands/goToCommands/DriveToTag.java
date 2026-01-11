package frc.robot.commands.goToCommands;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToTag extends Command {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Drive drive;
    private double dp;
    private double dtheta;

    // Defines PID controlelrs
    public DriveToTag(Drive drive, Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
        goToConstants.configurePID();
        this.robotPoseSupplier = robotPose;
        this.targetPoseSupplier = targetPose;
        this.drive = drive;
    }

    public DriveToTag(Drive drive, Supplier<Pose2d> targetPose) {
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
        dp = displacement.getNorm();
        double driveSpeed = goToConstants.driveController.calculate(displacement.getNorm());
        // sets velocity to the normalized displacement vector(direction of
        // displacement) timesd the desired drive speed.
        Translation2d setVelocity = displacement.div(dp).times(driveSpeed);

        // Gets the displacement angle -- if the robot rotates absolutely the wrong way,
        // switch target pose and robot pose.
        Rotation2d thetaDisplacement = targetPose.getRotation().minus(robotPose.getRotation());
        dtheta = thetaDisplacement.getRadians();
        // Gets PID for thera displacement
        double thetaVelocity = goToConstants.thetaController.calculate(dtheta);
        // runs velocities
        if (!robotPose.equals(new Pose2d())) {
            Logger.recordOutput("Commands/DriveToTag/SeeingTag", true);
            // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(setVelocity.getX(),
            // setVelocity.getY(), thetaVelocity, robotPose.getRotation()));
            drive.runVelocity(
                    new ChassisSpeeds(-setVelocity.getX(), -setVelocity.getY(), -thetaVelocity));
        } else {
            Logger.recordOutput("Commands/DriveToTag/SeeingTag", false);
            drive.runVelocity(
                    ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, robotPose.getRotation()));
        }
        // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(setVelocity.getX(),
        // setVelocity.getY(), thetaVelocity, robotPose.getRotation()));

        // Lets Log stuff
        Logger.recordOutput("Commands/DriveToTag/displacement", displacement);
        Logger.recordOutput("Commands/DriveToTag/RobotPose", robotPose);
        Logger.recordOutput("Commands/DriveToTag/Trajectory", displacement);
        Logger.recordOutput("Commands/DriveToTag/TargetPose", targetPose);
        Logger.recordOutput("Commands/DriveToTag/setDriveSpeed", driveSpeed);
        Logger.recordOutput("Commands/DriveToTag/setDriveVelocity", setVelocity);
        Logger.recordOutput("Commands/DriveToTag/thetaDifference", thetaDisplacement);
        Logger.recordOutput("Commands/DriveToTag/thetaVelocity", thetaVelocity);
    }

    @AutoLogOutput(key = "Commands/DriveToTag/atGoal")
    public boolean atGoal() {
        return Math.abs(dtheta) < goToConstants.thetaTolerance
                && Math.abs(dp) < goToConstants.driveTolerance;
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
