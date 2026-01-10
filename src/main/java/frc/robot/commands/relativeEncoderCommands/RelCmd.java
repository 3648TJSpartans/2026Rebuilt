package frc.robot.commands.relativeEncoderCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.absoluteEncoder.AbsEncoderConstants;
import frc.robot.subsystems.relativeEncoder.RelEncoder;

public class RelCmd extends Command {
    private final RelEncoder m_relEncoder;
    private final double position;

    public RelCmd(RelEncoder relEncoder, double position) {
        m_relEncoder = relEncoder;
        this.position = position;
        addRequirements(m_relEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("relEncoder/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("relEncoder/setPosition", position);
        Logger.recordOutput("relEncoder/position", m_relEncoder.getPosition());
        m_relEncoder.setTo(position);
    }

    @Override
    public void end(boolean interrupted) {
        m_relEncoder.stop();
        Logger.recordOutput("relEncoder/CommandScheduled", "Unscheduled");
    }

    @Override
    public boolean isFinished() {
        /*
         * A finish state is essential to commands that are used in command groups
         * (see command group file for more details). The code below finishes the
         * command if it's within a certain constant margin, the setpointTolerance.
         */
        return ((position - AbsEncoderConstants.setpointTolerance) < m_relEncoder.getPosition()
                &&
                m_relEncoder.getPosition() < (position + AbsEncoderConstants.setpointTolerance));
    }

}