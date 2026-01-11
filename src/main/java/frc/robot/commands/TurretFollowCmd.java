
package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class TurretFollowCmd extends Command {
    private final Turret m_turret;
    private final Supplier<Translation2d> targetDisplacementSupplier;
    private final Supplier<Rotation2d> robotRotationSupplier;

    @FunctionalInterface
    public interface Rotation2dSupplier extends Supplier<Rotation2d> {
    }

    @FunctionalInterface
    public interface Translation2dSupplier extends Supplier<Translation2d> {
    }

    @FunctionalInterface
    public interface Pose2dSupplier extends Supplier<Pose2d> {
    }

    public TurretFollowCmd(Turret turret, Translation2dSupplier targetDisplacementSupplier,
            Rotation2dSupplier robotRotationSupplier) {
        this.m_turret = turret;
        this.targetDisplacementSupplier = targetDisplacementSupplier;
        this.robotRotationSupplier = robotRotationSupplier;
        addRequirements(m_turret);
    }

    public TurretFollowCmd(Turret turret, Pose2dSupplier targetPoseTransform) {
        this(turret, () -> targetPoseTransform.get().getTranslation(),
                () -> targetPoseTransform.get().getRotation());
    }

    public TurretFollowCmd(Turret turret, Pose2dSupplier robotPoseSupplier,
            Translation2dSupplier targetPoseSupplier) {
        this(turret, () -> targetPoseSupplier.get().minus(robotPoseSupplier.get().getTranslation()),
                () -> robotPoseSupplier.get().getRotation());
    }


    @Override
    public void initialize() {
        Logger.recordOutput("Commands/TurretCommand/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Translation2d displacement = targetDisplacementSupplier.get();
        Rotation2d targetAngle = displacement.getAngle();
        Rotation2d robotRotation = robotRotationSupplier.get();
        Rotation2d totalRotation =
                targetAngle.minus(robotRotation).plus(TurretConstants.rotationOffset);

        double setPoint =
                (totalRotation.getRadians() * TurretConstants.encoderPositionFactor / (2 * Math.PI)
                        + .5);
        m_turret.setPosition(setPoint);

        Logger.recordOutput("Commands/TurretCommand//TargetTranslation", displacement);
        Logger.recordOutput("Commands/TurretCommand//TargetAngle", targetAngle);
        Logger.recordOutput("Commands/TurretCommand//RobotRotation", robotRotation);
        Logger.recordOutput("Commands/TurretCommand//totalRotation", totalRotation);
        Logger.recordOutput("Commands/TurretCommand//setPoint", setPoint);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        Logger.recordOutput("Commands/TurretCommand/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        /*
         * A finish state is essential to commands that are used in command groups (see command
         * group file for more details). The code below finishes the command if it's within a
         * certain constant margin, the setpointTolerance.
         */
        return false;
        // return ((position - TurretConstants.setpointTolerance) < m_turret.getPosition()
        // && m_turret.getPosition() < (position + TurretConstants.setpointTolerance));
    }

}